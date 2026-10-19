"""One charging adapter = one DPM86XX buck converter + one MFRC522 RFID reader.

State machine, advanced by one ``step()`` per control tick::

    OFF --setup--> IDLE --valid tag--> REQUESTING --setpoints sent--> SET_VALUES
                    ^                    ^   |                            | read-back ok, output ON
                    | tag removed        |   | pause() (source lost)      v
                  CUTOFF <-- charge done-+---+------------------------ CHARGING
    any state --error--> ERROR --retry setup--> IDLE

Error policy: low-level I/O methods raise; ``step()`` is the single place that
catches, and it always answers an unexpected error by switching the output off.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import math
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Optional, Sequence, TypeVar

from mfrc522 import MFRC522

from drivers.dpm86xx import (
    BaudRate,
    DPM86XXConfig,
    DPM86XXDevice,
    DPM86XXError,
    Protocol,
    RegulationMode,
    SaveSlot,
)

logger = logging.getLogger(__name__)

T = TypeVar("T")

TAG_DATA_BLOCK = 4  # block that holds the charging request
TAG_BLOCK_SIZE = 16  # 15 payload bytes + 1 checksum byte
MFRC522_VERSIONS = (0x91, 0x92)

# DPM86XX wire resolution: 0.01 V / 0.001 A
_V_RESOLUTION = 0.01
_I_RESOLUTION = 0.001

class AdapterState(IntEnum):
    OFF = 0
    IDLE = 1
    REQUESTING = 2
    SET_VALUES = 3
    CHARGING = 4
    CUTOFF = 5      # charge finished, output off, waiting for the tag to be removed
    ERROR = 6       # output off, retrying setup periodically


class AdapterError(RuntimeError):
    """Adapter-level failure that must put the adapter into the ERROR state."""


class TagDataError(ValueError):
    """The RFID tag could not be read, or its content is invalid / out of range."""


@dataclass(frozen=True, slots=True)
class RequestedValues:
    """Charging request stored on the robot's RFID tag (block 4, 16 bytes).

    Layout: [0] robot id, [1] tag id, [2:4] voltage mV, [4:6] max current mA,
    [6:8] cutoff current mA (all big-endian uint16), [8] battery type,
    [9:15] reserved, [15] checksum = sum(bytes 0..14) % 256.
    """
    robot_id: int
    tag_id: int
    voltage_v: float
    current_a: float
    cutoff_a: float
    battery_type: int

    @classmethod
    def from_tag_bytes(cls, data: Optional[Sequence[int]]) -> "RequestedValues":
        if not data or len(data) < TAG_BLOCK_SIZE:
            raise TagDataError(f"tag returned too little data: {data!r}")
        raw = bytes(data[:TAG_BLOCK_SIZE])

        calculated = sum(raw[:15]) % 256
        if calculated != raw[15]:
            raise TagDataError(
                f"checksum mismatch (calculated {calculated}, stored {raw[15]}): {raw.hex()}"
            )
        return cls(
            robot_id=raw[0],
            tag_id=raw[1],
            voltage_v=int.from_bytes(raw[2:4], "big") / 1000.0,
            current_a=int.from_bytes(raw[4:6], "big") / 1000.0,
            cutoff_a=int.from_bytes(raw[6:8], "big") / 1000.0,
            battery_type=raw[8],
        )

    @property
    def power_w(self) -> float:
        return self.voltage_v * self.current_a

    def validate(self, max_voltage_v: float, max_current_a: float) -> None:
        """Reject requests the hardware must never be asked for.

        The tag is untrusted input (a 1-byte checksum is no protection against
        a wrongly written or tampered tag), so every value is range-checked.
        """
        if not math.isfinite(self.voltage_v) or not 0 < self.voltage_v <= max_voltage_v:
            raise TagDataError(f"voltage {self.voltage_v} V outside (0, {max_voltage_v}] V")
        if not math.isfinite(self.current_a) or not 0 < self.current_a <= max_current_a:
            raise TagDataError(f"current {self.current_a} A outside (0, {max_current_a}] A")
        if not math.isfinite(self.cutoff_a):
            raise TagDataError(f"cutoff current must be finite, got {self.cutoff_a!r}")
        if not 0 <= self.cutoff_a < self.current_a:
            raise TagDataError(f"cutoff {self.cutoff_a} A must be >= 0 and below the charge current")

    def __str__(self) -> str:
        return (
            "=================== ROBO-TAG =====================\n"
            f"Robot ID:          #{self.robot_id}\n"
            f"Tag ID (Position): #{self.tag_id}\n"
            f"Voltage:           {self.voltage_v:.3f} V\n"
            f"Max. current:      {self.current_a:.3f} A\n"
            f"Cutoff current:    {self.cutoff_a:.3f} A\n"
            f"Battery type:      {self.battery_type}\n"
            "=================================================="
        )


class AdapterManager:
    def __init__(
            self,
            *,
            dpm_tty_port: str,
            dpm_baud_rate: BaudRate,
            dpm_address: int,
            dpm_stale_time: float,
            dpm_max_voltage: float,
            dpm_max_current: float,
            mfrc_spi_bus: int,
            mfrc_chip_select: int,
            mfrc_baud_rate: int,
            mfrc_pin_rst: int,
            mfrc_pin_mode: int,
            dpm_lock: asyncio.Lock,
            mfrc_lock: asyncio.Lock,
            tolerance: float = 0.01,
            limit_margin: float = 0.05,
            cutoff_hold_s: float = 5.0,
            max_setpoint_attempts: int = 3,
            tag_absent_polls: int = 3,
            error_retry_s: float = 10.0,
    ) -> None:
        """
        :param dpm_lock:  shared by every adapter on the same RS-485 bus
        :param mfrc_lock: shared by every adapter on the same SPI bus
        :param mfrc_pin_mode: GPIO numbering (``GPIO.BCM``) - must match the rest of the program
        :param tolerance: relative tolerance when comparing set-points with the request
        :param limit_margin: relative overshoot of the programmed limits tolerated before aborting
        :param cutoff_hold_s: current must stay below the cutoff this long before charging ends
        """
        if (not math.isfinite(dpm_max_voltage) or not math.isfinite(dpm_max_current)
                or dpm_max_voltage <= 0 or dpm_max_current <= 0):
            raise ValueError("DPM maximum voltage and current must be finite and positive")
        if not math.isfinite(dpm_stale_time) or dpm_stale_time <= 0:
            raise ValueError("dpm_stale_time must be finite and positive")
        if not math.isfinite(tolerance) or not 0 <= tolerance < 1:
            raise ValueError("tolerance must be finite and in [0, 1)")
        if not math.isfinite(limit_margin) or limit_margin < 0:
            raise ValueError("limit_margin must be finite and non-negative")
        if (not math.isfinite(cutoff_hold_s) or cutoff_hold_s < 0
                or max_setpoint_attempts < 1 or tag_absent_polls < 1
                or not math.isfinite(error_retry_s) or error_retry_s < 0):
            raise ValueError("invalid state-machine timing / retry parameter")

        self.dpm_max_voltage = dpm_max_voltage
        self.dpm_max_current = dpm_max_current
        self.stale_time = dpm_stale_time
        self.tolerance = tolerance
        self.limit_margin = limit_margin
        self.cutoff_hold_s = cutoff_hold_s
        self.max_setpoint_attempts = max_setpoint_attempts
        self.tag_absent_polls = tag_absent_polls
        self.error_retry_s = error_retry_s

        self._dpm_lock = dpm_lock
        self._mfrc_lock = mfrc_lock
        self._mfrc_args = dict(
            bus=mfrc_spi_bus, device=mfrc_chip_select, spd=mfrc_baud_rate,
            pin_mode=mfrc_pin_mode, pin_rst=mfrc_pin_rst,
        )
        self._mfrc: Optional[MFRC522] = None
        self._defaults_saved = False

        dpm_cfg = DPM86XXConfig(
            port=dpm_tty_port,
            baud_rate=dpm_baud_rate,
            address=dpm_address,
            protocol=Protocol.SIMPLE,
            stale_time=dpm_stale_time,
        )
        self.dpm_device = DPM86XXDevice(dpm_cfg, name=f"DPM8624-{dpm_address}")

        self.state = AdapterState.OFF
        self.requested_values: Optional[RequestedValues] = None

        # state-machine bookkeeping
        self._attempts = 0
        self._absent_polls = 0
        self._below_cutoff_since: Optional[float] = None
        self._error_since = 0.0
        self._output_off_confirmed = False
        self._last_rfid_problem: Optional[str] = None

        self._set_voltage: float = 0
        self._set_current: float = 0
        self._allocated_power_w: Optional[float] = None

    # ------------------------------------------------------------------ properties

    @property
    def dpm_address(self) -> int:
        return self.dpm_device.address


    @property
    def name(self) -> str:
        return f"adapter-{self.dpm_address}"


    @property
    def requested_power_w(self) -> float:
        return self.requested_values.power_w if self.requested_values else 0.0


    @property
    def committed_power_w(self) -> float:
        """Power this adapter is currently allowed to draw (0 unless it is starting or charging)."""
        if self.state in (AdapterState.SET_VALUES, AdapterState.CHARGING):
            return self.requested_power_w
        return 0.0


    # ------------------------------------------------------------------ blocking I/O helper

    async def _run_blocking(self, func: Callable[[], T], lock: asyncio.Lock) -> T:
        """Run blocking bus I/O in a worker thread while holding the bus lock.

        The lock is held until the thread has really finished - even if the
        caller is cancelled - so two threads can never talk on one bus at once.
        """
        async with lock:
            task = asyncio.ensure_future(asyncio.to_thread(func))
            try:
                return await asyncio.shield(task)
            except asyncio.CancelledError:
                await asyncio.wait({task})
                if not task.cancelled():
                    task.exception()  # mark as retrieved
                raise

    # ------------------------------------------------------------------ setup / shutdown

    async def setup_device(self, new_address: Optional[int] = None) -> bool:
        """Put the DPM into a safe default state and check the RFID reader.

        Safe state: output OFF, power-on default OFF, fast discharge OFF, 0 V / 0 A.
        Returns True on success (-> IDLE); on failure the adapter enters ERROR.
        """

        def _dpm_setup() -> None:
            with self.dpm_device.api as api:
                api.set_output(False)
                api.set_power_on_default(False)
                api.set_fast_discharge(False)
                api.set_voltage_and_current(0, 0)
                if not self._defaults_saved:
                    api.save_settings(SaveSlot.M0)
                    self._defaults_saved = True
                if new_address is not None:
                    # Must come last: after this the device only answers to the new address.
                    api.set_slave_address(new_address)
                    self.dpm_device.config.address = new_address

        def _mfrc_setup() -> None:
            if self._mfrc is None:
                self._mfrc = MFRC522(**self._mfrc_args)
            version = self._mfrc.Read_MFRC522(self._mfrc.VersionReg)
            if version not in MFRC522_VERSIONS:
                raise AdapterError(
                    f"RFID reader (CS {self._mfrc_args['device']}) not responding, version register = {version!r}"
                )

        try:
            await self._run_blocking(_dpm_setup, self._dpm_lock)
            await self._run_blocking(_mfrc_setup, self._mfrc_lock)
        except Exception:
            logger.exception("%s: setup failed", self.name)
            self._enter_error()
            return False

        self.requested_values = None
        self._allocated_power_w = None
        self._set_state(AdapterState.IDLE)
        logger.info("%s: initialised to safe default state", self.name)
        return True

    async def shutdown(self) -> None:
        """Switch the output off and release the SPI device (program exit)."""
        try:
            await self.set_output_state(False)
        except Exception:
            logger.critical("%s: could not disable output during shutdown!", self.name, exc_info=True)
        self.requested_values = None
        self._set_state(AdapterState.OFF)
        mfrc, self._mfrc = self._mfrc, None
        if mfrc is not None:
            with contextlib.suppress(Exception):
                mfrc.spi.close()

    # ------------------------------------------------------------------ DPM access

    async def set_output_state(self, enabled: bool) -> None:
        """Switch the DPM output on/off. Raises on communication failure."""

        def _apply() -> None:
            with self.dpm_device.api as api:
                api.set_output(enabled)

        await self._run_blocking(_apply, self._dpm_lock)
        logger.info("%s: output %s", self.name, "ON" if enabled else "OFF")

    async def _apply_setpoints(self, voltage_v: float, current_a: float) -> None:
        """Program voltage and current limit after checking them against the hardware limits."""
        if not 0 < voltage_v <= self.dpm_max_voltage:
            raise AdapterError(f"voltage {voltage_v:.2f} V exceeds limit {self.dpm_max_voltage} V")
        if not 0 < current_a <= self.dpm_max_current:
            raise AdapterError(f"current {current_a:.3f} A exceeds limit {self.dpm_max_current} A")

        def _apply() -> None:
            with self.dpm_device.api as api:
                api.set_voltage_and_current(voltage_v, current_a)

        await self._run_blocking(_apply, self._dpm_lock)
        logger.info("%s: set-points %.2f V / %.3f A (%.1f W)",
                    self.name, voltage_v, current_a, voltage_v * current_a)

    async def read_dpm_state(self) -> bool:
        """Refresh ``dpm_device.state``. Returns False (and keeps the old, ageing state) on failure."""
        try:
            await self._run_blocking(self.dpm_device.update_state, self._dpm_lock)
            return True
        except (DPM86XXError, OSError) as exc:  # pyserial's SerialException is an OSError
            logger.warning("%s: DPM read failed: %s", self.name, exc)
            return False

    # ------------------------------------------------------------------ RFID access

    def _read_tag_sync(self) -> Optional[RequestedValues]:
        mfrc = self._mfrc
        if mfrc is None:
            raise AdapterError("RFID reader not initialised")

        status, _tag_type = mfrc.MFRC522_Request(mfrc.PICC_REQIDL)
        if status != mfrc.MI_OK:
            return None  # no tag in the field - the normal idle case, not an error

        status, uid = mfrc.MFRC522_Anticoll()
        if status != mfrc.MI_OK:
            raise TagDataError("anti-collision failed")

        mfrc.MFRC522_SelectTag(uid)
        try:
            data = mfrc.MFRC522_Read(TAG_DATA_BLOCK)
        finally:
            mfrc.MFRC522_StopCrypto1()

        values = RequestedValues.from_tag_bytes(data)
        values.validate(self.dpm_max_voltage, self.dpm_max_current)
        return values

    async def read_rfid(self) -> Optional[RequestedValues]:
        """Return the request on the tag, or None if there is no (valid) tag. Never raises."""
        try:
            values = await self._run_blocking(self._read_tag_sync, self._mfrc_lock)
        except Exception as exc:
            message = str(exc)
            if message != self._last_rfid_problem:  # don't repeat the same warning every tick
                logger.warning("%s: RFID read problem: %s", self.name, message)
                self._last_rfid_problem = message
            return None
        self._last_rfid_problem = None
        return values

    async def _tag_removed(self) -> bool:
        """Poll the reader; True once no tag was seen for ``tag_absent_polls`` polls in a row."""
        values = await self.read_rfid()
        if values is None and self._last_rfid_problem is None:
            self._absent_polls += 1
        else:
            self._absent_polls = 0
        return self._absent_polls >= self.tag_absent_polls

    # ------------------------------------------------------------------ checks on the cached DPM state

    def _require_request(self) -> RequestedValues:
        if self.requested_values is None:
            raise AdapterError("no charging request available")
        return self.requested_values

    def _close(self, actual: Optional[float], expected: float, abs_tol: float) -> bool:
        return actual is not None and math.isclose(
            actual, expected, rel_tol=self.tolerance, abs_tol=abs_tol
        )

    def setpoints_match_request(self) -> bool:
        """True if the DPM reports exactly the requested voltage / current limit."""
        state = self.dpm_device.state
        return (self._close(state.voltage_set, self._set_voltage, _V_RESOLUTION)
                and self._close(state.current_set, self._set_current, _I_RESOLUTION))

    def output_within_limits(self) -> bool:
        """True if the measured output does not exceed the programmed limits.

        Only the upper bounds are checked: during the constant-current phase the
        measured voltage is *legitimately* below the set voltage (the old
        ``test_dpm_real_values`` treated that as an error).
        """
        s = self.dpm_device.state
        if None in (s.voltage_set, s.current_set, s.voltage_measured, s.current_measured):
            return False
        if s.voltage_measured < 0 or s.current_measured < -_I_RESOLUTION:
            return False
        return (s.voltage_measured <= s.voltage_set * (1 + self.limit_margin) + _V_RESOLUTION
                and s.current_measured <= s.current_set * (1 + self.limit_margin) + _I_RESOLUTION)

    def charge_complete(self) -> bool:
        """CC/CV charge is done when the DPM is in CV mode and the current stays below the cutoff.

        The hold time avoids ending the charge on a momentary dip - e.g. at start-up,
        when the current is still 0 A while the output ramps.
        """
        req = self._require_request()
        s = self.dpm_device.state
        if (s.regulation_mode is not RegulationMode.CV
                or s.current_measured is None
                or s.current_measured >= req.cutoff_a):
            self._below_cutoff_since = None
            return False

        now = time.monotonic()
        if self._below_cutoff_since is None:
            self._below_cutoff_since = now
        return now - self._below_cutoff_since >= self.cutoff_hold_s

    # ------------------------------------------------------------------ state machine

    def _set_state(self, new_state: AdapterState) -> None:
        if new_state is not self.state:
            logger.info("%s: %s -> %s", self.name, self.state.name, new_state.name)
            self.state = new_state
        self._absent_polls = 0

    def _enter_error(self, output_off: bool = False) -> None:
        self._error_since = time.monotonic()
        self._output_off_confirmed = output_off
        self._set_state(AdapterState.ERROR)

    async def _fail_safe(self) -> None:
        """Switch the output off and enter ERROR. Called for every unexpected error."""
        self.requested_values = None
        self._allocated_power_w = None
        confirmed = False
        try:
            await self.set_output_state(False)
            confirmed = True
        except Exception:
            logger.critical("%s: FAILED to disable output - retrying every tick", self.name, exc_info=True)
        self._enter_error(output_off=confirmed)

    async def pause(self, reason: str) -> None:
        """Switch the output off but keep the request; charging resumes via REQUESTING when allowed again."""
        if self.state not in (AdapterState.SET_VALUES, AdapterState.CHARGING):
            return
        logger.warning("%s: pausing charge: %s", self.name, reason)
        try:
            await self.set_output_state(False)
        except Exception:
            await self._fail_safe()
            return
        self._set_state(AdapterState.REQUESTING)

    async def step(self, *, allow_start: bool = True) -> None:
        """Advance the state machine by one tick.

        :param allow_start: set by the EnergyManager; False while no power source is
                            selected or the power budget is exhausted.
        """
        try:
            match self.state:
                case AdapterState.OFF:
                    pass
                case AdapterState.IDLE:
                    await self._step_idle()
                case AdapterState.REQUESTING:
                    await self._step_requesting(allow_start)
                case AdapterState.SET_VALUES:
                    await self._step_set_values()
                case AdapterState.CHARGING:
                    await self._step_charging()
                case AdapterState.CUTOFF:
                    await self._step_cutoff()
                case AdapterState.ERROR:
                    await self._step_error()
        except asyncio.CancelledError:
            with contextlib.suppress(Exception):
                await asyncio.shield(self._fail_safe())
            raise
        except Exception:
            logger.exception("%s: error in state %s - failing safe", self.name, self.state.name)
            await self._fail_safe()

    async def _step_idle(self) -> None:
        values = await self.read_rfid()
        if values is None:
            return
        self.requested_values = values
        self._allocated_power_w = None
        logger.info("%s: charging request received\n%s", self.name, values)
        self._set_state(AdapterState.REQUESTING)

    async def _step_requesting(self, allow_start: bool) -> None:
        req = self._require_request()
        if not allow_start or self._allocated_power_w == 0.0:
            # Waiting for power: give up if the robot has left in the meantime.
            if await self._tag_removed():
                self.requested_values = None
                self._allocated_power_w = None
                self._set_state(AdapterState.IDLE)
            return

        target_voltage = req.voltage_v
        target_current = min(
            req.current_a,
            self.dpm_max_current,
        )
        if target_current < (_I_RESOLUTION / 2):
            self._allocated_power_w = 0.0
            return
        self._attempts = 0
        await self._apply_setpoints(target_voltage, target_current)
        self._set_voltage = target_voltage
        self._set_current = target_current
        self._attempts = 0
        self._set_state(AdapterState.SET_VALUES)

    async def _step_set_values(self) -> None:
        if not await self.read_dpm_state():
            if self.dpm_device.state.is_stale(self.stale_time):
                raise AdapterError(f"no fresh DPM data for more than {self.stale_time} s")
            return

        if self.setpoints_match_request():
            await self.set_output_state(True)
            self._below_cutoff_since = None
            self._set_state(AdapterState.CHARGING)
            return

        self._attempts += 1
        if self._attempts >= self.max_setpoint_attempts:
            raise AdapterError("DPM did not accept the allocated set-points")
        await self._apply_setpoints(self._set_voltage, self._set_current)

    async def reset_set_values(self, set_power:float) -> None:
        req = self._require_request()
        if not math.isfinite(set_power) or set_power < 0:
            raise AdapterError(f"invalid power allocation: {set_power!r} W")

        # An allocation cannot authorize more than the power requested by the tag.
        allocated_power = min(float(set_power), req.power_w)
        target_voltage = req.voltage_v
        target_current = allocated_power / target_voltage
        target_current = min(target_current, req.current_a, self.dpm_max_current)

        target_current = math.floor(target_current / _I_RESOLUTION + 1e-12) * _I_RESOLUTION
        self._allocated_power_w = allocated_power

        # The DPM API rounds current to 1 mA. Do not enable an output for a target
        # that would round to zero, and turn off any output that was previously active.
        if target_current < (_I_RESOLUTION / 2):
            self._allocated_power_w = 0.0  # CHANGED: do not let _step_requesting fall back to full power.
            if self.state in (AdapterState.SET_VALUES, AdapterState.CHARGING):
                await self.pause("power allocation is zero or below DPM resolution")
            return

        # Avoid toggling the physical output on every control tick when the
        # allocation has not changed by at least one DPM set-point step.
        same_target = (
            math.isclose(self._set_voltage, target_voltage, abs_tol=_V_RESOLUTION / 2)
            and math.isclose(self._set_current, target_current, abs_tol=_I_RESOLUTION / 2)
        )
        if same_target and self.state in (AdapterState.SET_VALUES, AdapterState.CHARGING):
            return

        if self.state is AdapterState.CHARGING:
            await self.set_output_state(False)
            self._set_state(AdapterState.REQUESTING)

        await self._apply_setpoints(target_voltage, target_current)
        self._set_voltage = target_voltage
        self._set_current = target_current
        self._attempts = 0
        self._below_cutoff_since = None
        self._set_state(AdapterState.SET_VALUES)

    async def _step_charging(self) -> None:
        # A failed read is tolerated until the cached state becomes stale.
        read_ok = await self.read_dpm_state()
        s = self.dpm_device.state
        if not read_ok:
            if s.is_stale(self.stale_time):
                raise AdapterError(f"no fresh DPM data for more than {self.stale_time} s")
            return
        if s.is_stale(self.stale_time):
            raise AdapterError(f"no fresh DPM data for more than {self.stale_time} s")
        if not s.output_enabled:
            raise AdapterError("DPM output switched off unexpectedly (protection tripped?)")
        if not self.output_within_limits():
            raise AdapterError(
                f"measured output ({s.voltage_measured} V / {s.current_measured} A) exceeds the programmed limits"
            )
        if self.charge_complete():
            await self.set_output_state(False)
            logger.info("%s: charge complete (current below %.3f A)", self.name, self._require_request().cutoff_a)
            self._set_state(AdapterState.CUTOFF)

    async def _step_cutoff(self) -> None:
        # Stay here until the robot leaves, otherwise a full battery would restart the cycle.
        if await self._tag_removed():
            self.requested_values = None
            self._allocated_power_w = None
            self._set_state(AdapterState.IDLE)

    async def _step_error(self) -> None:
        if not self._output_off_confirmed:
            try:
                await self.set_output_state(False)
                self._output_off_confirmed = True
            except Exception as exc:
                logger.debug("%s: output-off retry failed: %s", self.name, exc)
                return
        if time.monotonic() - self._error_since >= self.error_retry_s:
            logger.info("%s: retrying setup", self.name)
            await self.setup_device()



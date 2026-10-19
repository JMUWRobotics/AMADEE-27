# -*- coding: utf-8 -*-
"""VE.Direct reader for a Victron SmartSolar MPPT 150/35.

The blocking vedirect-pkg reader runs in a daemon thread; the most recent
packet is exposed to asyncio code through :meth:`VedirectManager.get_latest_state`,
:meth:`get_fresh_state` and :meth:`wait_for_state`.
"""
from __future__ import annotations

import asyncio
import logging
import threading
import time
from dataclasses import dataclass, field
from enum import KEEP, Flag, IntEnum
from typing import Mapping, Optional, Any

from vedirect_m8.ve_controller import VedirectController

logger = logging.getLogger(__name__)


# =============================================================================
# ENUMS
# =============================================================================

class _TolerantIntEnum(IntEnum):
    """IntEnum that maps codes it does not know to ``UNKNOWN`` instead of raising.

    Victron adds codes with new firmware.  Raising on an unknown value would
    drop the whole packet - including a fault code that we most want to see.
    """

    @classmethod
    def _missing_(cls, value):
        return cls.UNKNOWN  # type: ignore[attr-defined]


class MpptOffReason(Flag, boundary=KEEP):
    """Bit field 'OR'.  ``boundary=KEEP`` tolerates bits added by newer firmware."""
    NO_INPUT_POWER = 0x00000001
    SWITCHED_OFF_POWER_SWITCH = 0x00000002
    SWITCHED_OFF_DEVICE_MODE = 0x00000004
    REMOTE_INPUT = 0x00000008
    PROTECTION_ACTIVE = 0x00000010
    PAYGO = 0x00000020
    BMS = 0x00000040
    ENGINE_SHUTDOWN_DETECTION = 0x00000080
    ANALYSING_INPUT_VOLTAGE = 0x00000100


class MpptOperationState(_TolerantIntEnum):
    """Field 'CS'."""
    UNKNOWN = -1
    OFF = 0
    FAULT = 2
    BULK = 3
    ABSORPTION = 4
    FLOAT = 5
    STORAGE = 6
    EQUALIZE_MANUAL = 7
    STARTING_UP = 245
    REPEATED_ABSORPTION = 246
    AUTO_EQUALIZE_RECONDITION = 247
    BATTERY_SAFE = 248
    EXTERNAL_CONTROL = 252


class MpptError(_TolerantIntEnum):
    """Field 'ERR'."""
    UNKNOWN = -1
    NO_ERROR = 0
    BATTERY_VOLTAGE_TOO_HIGH = 2
    CHARGER_TEMPERATURE_TOO_HIGH = 17
    CHARGER_OVER_CURRENT = 18
    CHARGER_CURRENT_REVERSED = 19
    BULK_TIME_LIMIT_EXCEEDED = 20
    CURRENT_SENSOR_ISSUE = 21  # sensor bias / sensor broken
    TERMINALS_OVERHEATED = 26
    CONVERTER_ISSUE = 28
    INPUT_VOLTAGE_TOO_HIGH = 33  # solar panel
    INPUT_CURRENT_TOO_HIGH = 34  # solar panel
    INPUT_SHUTDOWN_EXCESSIVE_BATTERY_VOLTAGE = 38
    INPUT_SHUTDOWN_CURRENT_FLOW_DURING_OFF_MODE = 39
    LOST_COMMUNICATION_WITH_ONE_OF_DEVICES = 65
    SYNCHRONISED_CHARGING_DEVICE_CONFIGURATION_ISSUE = 66
    BMS_CONNECTION_LOST = 67
    NETWORK_MISCONFIGURED = 68
    CPU_TEMPERATURE_TOO_HIGH = 114
    FACTORY_CALIBRATION_DATA_LOST = 116
    INVALID_INCOMPATIBLE_FIRMWARE = 117
    USER_SETTINGS_INVALID = 119


class MpptTrackerOperationState(_TolerantIntEnum):
    """Field 'MPPT'."""
    UNKNOWN = -1
    OFF = 0
    VOLTAGE_OR_CURRENT_LIMITED = 1
    MPP_TRACKER_ACTIVE = 2


class MpptFaultError(RuntimeError):
    """The charge controller reports a fault."""


# =============================================================================
# STATE SNAPSHOT + PARSING
# =============================================================================

@dataclass(frozen=True, slots=True)
class MpptState:
    """Immutable snapshot of one VE.Direct text block."""
    voltage_bus: float  # [V]   battery / bus voltage
    voltage_panel: float  # [V]   solar panel voltage
    power_panel: float  # [W]   solar panel power
    current_bus: float  # [A]   battery / bus current
    relay_state: bool  # normal operation = off, battery low voltage = on
    off_reason: MpptOffReason  # why the unit is off
    yield_total: float  # [kWh]
    yield_today: float  # [kWh]
    maximum_power_today: float  # [W]
    yield_yesterday: float  # [kWh]
    maximum_power_yesterday: float  # [W]
    error_code: MpptError
    state_of_operation: MpptOperationState
    firmware_version: str
    product_id: str
    serial_number: str
    day_sequence: int
    tracker_operation_mode: MpptTrackerOperationState
    received_at: float = field(default_factory=time.monotonic)  # time.monotonic()

    @property
    def age_s(self) -> float:
        """Seconds since this packet was received."""
        return time.monotonic() - self.received_at


# Fields the controller sends in every block; a packet without them is unusable.
_REQUIRED_KEYS = ("V", "VPV", "PPV", "I", "CS", "ERR")

def parse_mppt_packet(packet: Mapping[str, str]) -> MpptState:
    """Convert a raw VE.Direct dict into an :class:`MpptState`.

    Safety-relevant fields (voltages, power, state, error) are required: a
    missing key raises ``KeyError`` instead of silently reading as 0.
    Purely informational fields (yield history, ids) fall back to defaults.
    """
    missing = [key for key in _REQUIRED_KEYS if key not in packet]
    if missing:
        raise KeyError(f"packet is missing required field(s): {', '.join(missing)}")

    def number(key: str, divisor: float = 1.0) -> float:
        return float(packet.get(key, 0)) / divisor

    return MpptState(
        voltage_bus=number("V", 1000.0),  # mV -> V
        voltage_panel=number("VPV", 1000.0),  # mV -> V
        power_panel=number("PPV"),  # W
        current_bus=number("I", 1000.0),  # mA -> A
        relay_state=packet.get("Relay") == "ON",
        off_reason=MpptOffReason(int(packet.get("OR", "0"), 16)),  # hex string, e.g. "0x00000001"
        yield_total=number("H19", 100.0),  # 0.01 kWh -> kWh
        yield_today=number("H20", 100.0),
        maximum_power_today=number("H21"),
        yield_yesterday=number("H22", 100.0),
        maximum_power_yesterday=number("H23"),
        error_code=MpptError(int(packet["ERR"])),
        state_of_operation=MpptOperationState(int(packet["CS"])),
        firmware_version=packet.get("FW", ""),
        product_id=packet.get("PID", ""),
        serial_number=packet.get("SER#", ""),
        day_sequence=int(packet.get("HSDS", 0)),
        tracker_operation_mode=MpptTrackerOperationState(int(packet.get("MPPT", 0))),
    )


# =============================================================================
# VEDIRECT MANAGER
# =============================================================================

class VedirectManager:
    def __init__(
            self,
            tty_port: str,
            timeout: float,
            serial_test: dict[Any, Any],
            *,
            reconnect_delay_s: float = 5.0,
    ) -> None:
        self._serial_conf = {"serial_port": tty_port, "timeout": timeout}
        self._serial_test = serial_test
        self._reconnect_delay_s = reconnect_delay_s

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._first_state = asyncio.Event()
        self._latest: Optional[MpptState] = None  # replaced atomically by the reader thread

    # ------------------------------------------------------------------ reader thread

    def _on_packet(self, packet: dict) -> None:
        """Called by vedirect-pkg in the reader thread for every complete block."""
        try:
            state = parse_mppt_packet(packet)
        except Exception as exc:  # a bad packet must never kill the reader
            logger.warning("Dropping malformed VE.Direct packet: %r", exc)
            return

        self._latest = state
        if self._loop is not None and not self._first_state.is_set():
            self._loop.call_soon_threadsafe(self._first_state.set)

    def _run_worker(self) -> None:
        """Reader loop: (re)connect forever until :meth:`stop` is called."""
        while not self._stop.is_set():
            try:
                controller = VedirectController(
                    serial_conf=self._serial_conf,
                    serial_test=self._serial_test,
                )
                logger.info("VE.Direct reader running on %s", self._serial_conf["serial_port"])
                controller.read_data_callback(self._on_packet)
                logger.warning("VE.Direct reader returned unexpectedly")
            except Exception:
                logger.exception("VE.Direct reader failed")
            self._stop.wait(self._reconnect_delay_s)  # returns early when stop() is called

    # ------------------------------------------------------------------ public API

    def start(self) -> None:
        """Start the background reader (non-blocking). Call from inside the running event loop."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._loop = asyncio.get_running_loop()
        self._stop.clear()
        # Daemon thread: the blocking serial read cannot be interrupted, and a
        # non-daemon thread would keep the process alive after shutdown.
        self._thread = threading.Thread(target=self._run_worker, name="vedirect-reader", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Ask the reader to stop reconnecting (the blocked read ends with the process)."""
        self._stop.set()

    def get_latest_state(self) -> Optional[MpptState]:
        """Most recent state, or ``None`` if nothing was received yet. May be old - see :meth:`get_fresh_state`."""
        return self._latest

    def get_fresh_state(self, max_age_s: float = 5.0) -> Optional[MpptState]:
        """Most recent state if it is younger than *max_age_s*, otherwise ``None``."""
        state = self._latest
        if state is None or state.age_s > max_age_s:
            return None
        return state

    async def wait_for_state(self, timeout: Optional[float] = None) -> MpptState:
        """Wait until the first packet has arrived and return the latest one.

        Raises ``TimeoutError`` if nothing arrives within *timeout* seconds.
        """
        await asyncio.wait_for(self._first_state.wait(), timeout)
        assert self._latest is not None
        return self._latest

    async def startup(self, timeout: float = 30.0) -> bool:
        """Wait for the controller to finish starting up and verify it is not in a fault state.

        Raises ``TimeoutError`` (no data / still starting) or :class:`MpptFaultError`.
        """
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout

        state = await self.wait_for_state(timeout)
        while state.state_of_operation is MpptOperationState.STARTING_UP:
            if loop.time() > deadline:
                raise TimeoutError("MPPT still in STARTING_UP state")
            await asyncio.sleep(1)
            state = self._latest or state

        if (state.state_of_operation is MpptOperationState.FAULT
                or state.error_code is not MpptError.NO_ERROR):
            raise MpptFaultError(
                f"MPPT fault: state={state.state_of_operation.name}, error={state.error_code.name}"
            )
        return True



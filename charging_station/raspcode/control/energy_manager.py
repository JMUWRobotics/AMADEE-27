"""System-level supervision: power-source selector, MPPT and all charging adapters."""
from __future__ import annotations

import asyncio
import contextlib
import logging
import math
from enum import IntEnum
from typing import Optional, Sequence

import RPi.GPIO as GPIO

from control.adapter_manager import AdapterManager, AdapterState
from control.vedirect_manager import VedirectManager, MpptOperationState, MpptTrackerOperationState, \
    MpptError
from drivers.dpm86xx import BaudRate

logger = logging.getLogger(__name__)

class PowerSelectorState(IntEnum):
    OFF = 0
    MPPT = 1
    POWER_SUPPLY = 2
    UNDEFINED = 3  # both feedback pins active - wiring fault or relay in transition


# =============================================================================
# FAIR POWER DIVISION (pure functions)
# =============================================================================

def allocate_max_min(demands: Sequence[float], budget: float) -> list[float]:
    """Max-min fair ("water-filling") division.

    Everybody gets at most an equal share; whoever asks for less than that gets exactly
    what was asked and the surplus is split among the others.
    ``[100, 400, 800]`` with 600 W  ->  ``[100, 250, 250]``
    """
    # Reject malformed numeric inputs before they can propagate into hardware set-points.
    if not math.isfinite(float(budget)) or any(not math.isfinite(float(d)) or d < 0 for d in demands):
        raise ValueError("power budget and demands must be finite; demands must be non-negative")
    alloc = [0.0] * len(demands)
    pending = [i for i, d in enumerate(demands) if d > 0]
    remaining = max(0.0, budget)
    while pending and remaining > 1e-9:
        share = remaining / len(pending)
        satisfied = [i for i in pending if demands[i] <= share]
        if not satisfied:  # nobody can be fully served: equal split
            for i in pending:
                alloc[i] = share
            break
        for i in satisfied:
            alloc[i] = demands[i]
            remaining -= demands[i]
        pending = [i for i in pending if i not in satisfied]
    return alloc


def allocate_proportional(demands: Sequence[float], budget: float) -> list[float]:
    """Everybody is scaled by the same factor: ``[100, 400, 800]`` with 650 W -> ``[50, 200, 400]``."""
    # Reject malformed numeric inputs before they can propagate into hardware set-points.
    if not math.isfinite(float(budget)) or any(not math.isfinite(float(d)) or d < 0 for d in demands):
        raise ValueError("power budget and demands must be finite; demands must be non-negative")
    total = sum(d for d in demands if d > 0)
    if total <= budget:
        return [max(0.0, d) for d in demands]
    return [max(0.0, d) * budget / total for d in demands]

def load_system_config(config: dict) -> tuple[list[AdapterManager], VedirectManager]:
    """Build all adapters and the MPPT reader from the parsed ``system.yaml``."""
    dpm_config = config["adapter"]
    devices = dpm_config["devices"]

    addresses = [d["dpm_address"] for d in devices]
    chip_selects = [d["mfrc_chip_select"] for d in devices]
    if len(set(addresses)) != len(addresses):
        raise ValueError(f"duplicate dpm_address in config: {addresses}")
    if len(set(chip_selects)) != len(chip_selects):
        raise ValueError(f"duplicate mfrc_chip_select in config: {chip_selects}")

    # One lock per physical bus, shared by all adapters on it
    # (all DPMs share one RS-485 line, all readers share one SPI bus).
    dpm_lock = asyncio.Lock()
    mfrc_lock = asyncio.Lock()

    adapters = [
        AdapterManager(
            dpm_tty_port=dpm_config["dpm_tty_port"],
            dpm_baud_rate=BaudRate(dpm_config["dpm_baudrate"]),
            dpm_address=dev["dpm_address"],
            dpm_stale_time=dpm_config["dpm_stale_time"],
            dpm_max_voltage=dev["dpm_max_voltage"],
            dpm_max_current=dev["dpm_max_current"],
            mfrc_spi_bus=dpm_config["mfrc_spi_bus"],
            mfrc_chip_select=dev["mfrc_chip_select"],
            mfrc_baud_rate=dpm_config["mfrc_baudrate"],
            mfrc_pin_rst=dpm_config["mfrc_pin_rst"],
            mfrc_pin_mode=GPIO.BCM,
            dpm_lock=dpm_lock,
            mfrc_lock=mfrc_lock,
        )
        for dev in devices
    ]

    mppt_config = config["mppt"]
    ve_device = VedirectManager(
        tty_port=mppt_config["tty_port"],
        timeout=mppt_config["timeout"],
        serial_test=mppt_config["serial_test"],
    )
    return adapters, ve_device


class EnergyManager:
    def __init__(self, config: dict) -> None:
        """Supervises the power-source selector, the MPPT and all charging adapters.

        :param config: parsed ``system.yaml``. Optional top-level keys:
                       ``max_total_power_w`` (budget over all adapters) and
                       ``poll_interval_s`` (control tick, default 0.5 s).
        """
        self.power_supply_pin: int = config["power_supply_pin"]
        self.mppt_pin: int = config["mppt_pin"]
        self.max_total_power_supply_w: Optional[float] = config["max_total_power_supply_w"]
        self.poll_interval_s: float = float(config.get("poll_interval_s", 0.5))

        self.adapters, self.mppt = load_system_config(config)
        self.power_selector_state = PowerSelectorState.OFF
        self.power_margin = float(config.get("power_margin", 0.10))
        self.allocation_policies = config.get("allocation_policies", "min_max")

        self.low_battery_voltage_v = float(config.get("low_battery_voltage_v", 51.0))
        self.low_battery_hysteresis_v = float(config.get("low_battery_hysteresis_v", 0.5))
        self.mppt_max_age_s = float(config.get("mppt_max_age_s", max(2.0, 3.0 * self.poll_interval_s)))
        self.mppt_startup_timeout_s = float(config.get("mppt_startup_timeout_s", 30.0))
        self._low_battery_latched = False
        self._gpio_ready = False

    # ------------------------------------------------------------------ setup / shutdown

    def _setup_gpio(self) -> None:
        # Every library that touches the pins (MFRC522 included) must use the same numbering.
        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BCM)
        elif GPIO.getmode() != GPIO.BCM:
            raise RuntimeError("GPIO already configured with a numbering other than BCM")
        GPIO.setup(self.power_supply_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.mppt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    async def setup_system(self) -> bool:
        """Initialise GPIO, all adapters and the MPPT. Returns True only if everything came up.

        Adapters that fail enter ERROR and retry on their own, so the system can run degraded.
        """
        try:
            await asyncio.to_thread(self._setup_gpio)
            self._gpio_ready = True
            await self._refresh_power_selector_state()
        except Exception:
            logger.exception("GPIO setup failed")
            return False

        if self.power_selector_state == PowerSelectorState.MPPT:
            self.mppt.start()  # without this the MPPT never delivers data and startup() would hang

        # gather(return_exceptions=True): one failing device must not cancel the others.
        jobs = {adp.name: adp.setup_device() for adp in self.adapters}
        if self.power_selector_state == PowerSelectorState.MPPT:
            jobs["mppt"] = self.mppt.startup()
        results = await asyncio.gather(*jobs.values(), return_exceptions=True)

        ok = True
        for label, result in zip(jobs, results):
            if isinstance(result, BaseException):
                logger.error("Setup of %s failed: %r", label, result)
                ok = False
            elif result is False:
                ok = False
        return ok

    async def shutdown(self) -> None:
        """Switch every output off and release the hardware. Always call this on exit."""
        await asyncio.gather(*(adp.shutdown() for adp in self.adapters), return_exceptions=True)
        if self.power_selector_state == PowerSelectorState.MPPT:
            self.mppt.stop()
        with contextlib.suppress(Exception):
            GPIO.cleanup()
        logger.info("Shutdown complete")

    # ------------------------------------------------------------------ power selector

    def read_power_selector_state(self) -> PowerSelectorState:
        """Read the (active-low) feedback pins of the source selector and cache the result."""
        mppt_active = GPIO.input(self.mppt_pin) == GPIO.LOW
        supply_active = GPIO.input(self.power_supply_pin) == GPIO.LOW

        if mppt_active and supply_active:
            state = PowerSelectorState.UNDEFINED
        elif mppt_active:
            state = PowerSelectorState.MPPT
        elif supply_active:
            state = PowerSelectorState.POWER_SUPPLY
        else:
            state = PowerSelectorState.OFF

        if state is not self.power_selector_state:
            logger.info("Power selector: %s -> %s", self.power_selector_state.name, state.name)
            self.power_selector_state = state
        return state


    async def _refresh_power_selector_state(self) -> PowerSelectorState:
        return await asyncio.to_thread(self.read_power_selector_state)

    async def _source_ok(self) -> bool:
        source_ok = self.power_selector_state in (PowerSelectorState.MPPT, PowerSelectorState.POWER_SUPPLY)

        if not source_ok:
            # Fail-safe: never keep charging from an undefined / disconnected source.
            await asyncio.gather(
                *(adp.pause(f"power selector is {self.power_selector_state.name}") for adp in self.adapters),
                return_exceptions=True,
            )

        return source_ok

    # ------------------------------------------------------------------ control loop
    def power_limit_reached(self) -> Optional[float]:
        """Return available charging watts, or None for an explicitly uncapped supply."""
        if self.power_selector_state is PowerSelectorState.POWER_SUPPLY:
            return self.max_total_power_supply_w
        state = self.mppt.get_fresh_state(self.mppt_max_age_s)
        if state is None:
            logger.warning("MPPT telemetry is missing/stale; charging budget set to 0 W")
            return 0.0
        if state.state_of_operation in (
                MpptOperationState.FAULT,
                MpptOperationState.UNKNOWN,
                MpptOperationState.OFF,
                MpptOperationState.STARTING_UP,
        ):
            logger.error("MPPT is not in a usable state: %s", state.state_of_operation.name)
            return 0.0
        if state.tracker_operation_mode == MpptTrackerOperationState.VOLTAGE_OR_CURRENT_LIMITED:
            logger.debug("MPP not reached; adapter requests are not software-capped")
            return None
        if state.tracker_operation_mode is MpptTrackerOperationState.UNKNOWN:
            logger.error("MPPT tracker mode is unknown; charging budget set to 0 W")
            return 0.0
        if state.error_code not in (MpptError.NO_ERROR,):
            logger.error("MPPT reports error: %s", state.error_code.name)
            return 0.0
        measured_bus_power = state.voltage_bus * state.current_bus
        return measured_bus_power * (1 - self.power_margin)

    def _adp_in_state(self, state: AdapterState) -> list[AdapterManager]:
        return [adp for adp in self.adapters if adp.state is state]

    def _adapters_needing_power(self) -> list[AdapterManager]:
        active_states = (AdapterState.REQUESTING, AdapterState.SET_VALUES, AdapterState.CHARGING)
        return [adp for adp in self.adapters if adp.state in active_states]

    async def _allocate_power(self, power_limit: Optional[float] = None) -> bool:
        """Apply a fair power budget to requests before a SET_VALUES state can enable output."""
        adapters = self._adapters_needing_power()
        if power_limit is None:
            # An uncapped external supply restores each active request to its
            # full RFID-authorised power after any previous MPPT-side throttling.
            results = await asyncio.gather(
                *(adp.reset_set_values(adp.requested_power_w) for adp in adapters),
                return_exceptions=True,
            )
            for adp, result in zip(adapters, results):
                if isinstance(result, BaseException):
                    logger.error("%s: could not restore full request on uncapped supply: %r", adp.name, result)
                    await adp.pause("failed to restore set-points on supply change")
                    # A pause failure invokes AdapterManager's own fail-safe policy.
            return True

        demands = [adp.requested_power_w for adp in adapters]
        allocator = allocate_proportional if self.allocation_policies == "proportional" else allocate_max_min
        allocations = allocator(demands, max(0.0, power_limit))
        results = await asyncio.gather(
            *(adp.reset_set_values(power_w) for adp, power_w in zip(adapters, allocations)),
            return_exceptions=True,
        )
        for adp, result, allocated in zip(adapters, results, allocations):
            if isinstance(result, BaseException):
                logger.error("%s: failed to apply %.2f W allocation: %r", adp.name, allocated, result)
                # Do not continue charging at the old, possibly too-large set-point.
                await adp.pause("power budget reallocation failed")
            else:
                logger.debug("%s: allocated %.2f W of %.2f W requested", adp.name, allocated, adp.requested_power_w)
        return power_limit > 0.0

    async def _trigger_steps(self, allow_start: bool):
        # Adapters are independent: one failing must not stop the others.
        results = await asyncio.gather(*[adp.step(allow_start=allow_start) for adp in self.adapters], return_exceptions=True)
        for adp, result in zip(self.adapters, results):
            if isinstance(result, BaseException):
                logger.error("%s: unhandled error in step(): %r", adp.name, result)

    async def step_idle(self) -> bool:
        await self._refresh_power_selector_state()
        await self._source_ok()
        await self._trigger_steps(allow_start=False)
        return any(adp.state in (AdapterState.REQUESTING, AdapterState.SET_VALUES, AdapterState.CHARGING)
                   for adp in self.adapters)

    async def step_charging(self) -> bool:
        await self._refresh_power_selector_state()
        source_ok = await self._source_ok()
        low_battery = await self.low_battery()
        if not source_ok or low_battery:
            await self._trigger_steps(allow_start=False)
            return any(adp.state in (AdapterState.REQUESTING, AdapterState.SET_VALUES, AdapterState.CHARGING)
                       for adp in self.adapters)

        power_limit = self.power_limit_reached()
        allow_start = await self._allocate_power(power_limit)
        await self._trigger_steps(allow_start=allow_start)
        return any(adp.state in (AdapterState.REQUESTING, AdapterState.SET_VALUES, AdapterState.CHARGING)
                   for adp in self.adapters)

    async def low_battery(self) -> bool:
        """Pause charging on a low MPPT bus; bypass this check on the external supply."""
        if self.power_selector_state is not PowerSelectorState.MPPT:
            self._low_battery_latched = False
            return False

        state = self.mppt.get_fresh_state(self.mppt_max_age_s)
        if state is None:
            await asyncio.gather(
                *(adp.pause("MPPT telemetry missing or stale") for adp in self.adapters),
                return_exceptions=True,
            )
            return True

        if self._low_battery_latched:
            self._low_battery_latched = state.voltage_bus < (
                self.low_battery_voltage_v + self.low_battery_hysteresis_v
            )
        else:
            self._low_battery_latched = state.voltage_bus <= self.low_battery_voltage_v

        if self._low_battery_latched:
            await asyncio.gather(
                *(adp.pause(f"MPPT bus voltage low ({state.voltage_bus:.2f} V)") for adp in self.adapters),
                return_exceptions=True,
            )
        return self._low_battery_latched




















import asyncio
import logging
import select
import signal
import sys
from pathlib import Path
from typing import Optional

import yaml

from raspcode.control.adapter_manager import AdapterState  
from raspcode.control.comm_manager import CommConfig, CommManager
from raspcode.control.debug_manager import DebugManager
from raspcode.control.energy_manager import EnergyManager
from raspcode.control.gpio_manager import GPIOManager
from raspcode.core.system_state import State 

logger = logging.getLogger("__name__")

class StateMachine:
    def __init__(self, config_file: Path) -> None:
        self.config_file = config_file
        self.config = None
        self.energy_manager: Optional[EnergyManager] = None
        self.debug_manager: Optional[DebugManager] = None
        self.gpio_manager: Optional[GPIOManager] = None
        self.comm: Optional[CommManager] = None
        self.state = State.BOOT
        self.poll_interval_s = 1.0  # main-loop period (was hard-coded in run())
        self._running = True

    async def run(self) -> None:
        """Main non-blocking async loop executing state handlers and transitions."""
        logger.info("Starting State Machine...")
        try:
            if self.state == State.BOOT:
                await self._handle_boot()
            while self._running:
                await asyncio.sleep(self.poll_interval_s)
                if self.gpio_manager is not None:  # None after a failed boot
                    self.gpio_manager.set_light(self.state)  # Update light state based on current state
                try:
                    # was: ``if self.energy_manager.low_battery():`` - a coroutine is always truthy, so the
                    # station was permanently in LOW_BATTERY; it also overwrote FAULT.
                    if self.energy_manager is not None:
                        low_battery = await self.energy_manager.low_battery()
                        if low_battery and self.state is not State.FAULT:
                            self.state = State.LOW_BATTERY
                    match self.state:
                        case State.IDLE:
                            await self._handle_idle()

                        case State.FAULT:
                            await self._handle_fault()

                        case State.LOW_BATTERY:
                            await self._handle_low_battery()

                        case State.LOW_SOLAR_IDLE:
                            await self._handle_low_solar_idle()

                        case State.LOW_SOLAR_CHARGING:
                            await self._handle_low_solar_charging()

                        case State.CHARGING:
                            await self._handle_charging()

                        case State.DEBUG:
                            await self._handle_debug()

                except asyncio.CancelledError:
                    logger.info("Shutdown requested.")
                    self._running = False
                    break
                except Exception as e:
                    logger.error(f"Unexpected error in StateMachine loop: {e}", exc_info=True)
                    self.state = State.FAULT
        finally:
            await self.shutdown()

    async def shutdown(self) -> None:
        """Close the WiFi protocol (every session ends, every port goes dead), then the hardware."""
        self._running = False
        if self.comm is not None:
            try:
                await self.comm.stop()
            except Exception:
                logger.critical("Could not stop the charging protocol cleanly", exc_info=True)
        if self.energy_manager is not None:
            try:
                await self.energy_manager.shutdown()
            except Exception:
                logger.critical("Could not shut down the energy manager cleanly", exc_info=True)

    async def _handle_boot(self) -> None:
        logger.info("Executing BOOT sequence...")
        try:
            with open(self.config_file, "r", encoding="utf-8") as f:
                self.config = yaml.safe_load(f)

            #self.debug_manager = DebugManager(charging_manager=self.charging_manager)
            self.gpio_manager = GPIOManager(config=self.config)
            power_selector_state = self.gpio_manager.read_power_selector_state()
            enable_switches = self.gpio_manager.read_switches()
            self.energy_manager = EnergyManager(config=self.config,power_selector_state=power_selector_state,enable_switches=enable_switches)  # Initial read to cache the state
            station_ready = await self.energy_manager.setup_system()

            if station_ready:
                await self._start_comm()
                logger.info("Boot sequence completed successfully. Switching to IDLE.")
                self.state = State.IDLE
            else:
                logger.error("Station setup failed during boot.")
                self.state = State.FAULT

        except Exception as e:
            logger.error(f"Boot failure: {e}", exc_info=True)
            self.state = State.FAULT

    async def _start_comm(self) -> None:
        """Start the WiFi charging protocol. Fail closed: once it is configured, no charge without it."""
        section = self.config.get("comm")
        if section is None:
            logger.warning("No 'comm' section in system.yaml - WiFi charging protocol is DISABLED, "
                           "an RFID tag alone starts a charge.")
            return
        comm_config = CommConfig.from_dict(section)
        if not comm_config.enabled:
            logger.warning("comm.enabled is false - WiFi charging protocol is DISABLED, "
                           "an RFID tag alone starts a charge.")
            return
        self.comm = CommManager(self.energy_manager, comm_config, lambda: self.state)
        await self.comm.start()  # raises (-> FAULT) if e.g. the port is taken; the gate then stays closed

    async def _handle_idle(self) -> None:
        adp_states = await self.energy_manager.step_idle()  # Non-blocking step to check for charging requests
        # Process the adapter states as needed
        for addr, state in adp_states.items():
            if state == AdapterState.IDLE:
                logger.debug(f"Adapter {addr} is idle and ready for charging requests.")
                continue
            elif state in (AdapterState.REQUESTING, AdapterState.SET_VALUES, AdapterState.CHARGING):
                logger.debug(f"Adapter {addr} is in state {state}. Transitioning to CHARGING.")
                self.state = State.CHARGING
                break

    async def _handle_charging(self) -> None:
        adp_states = await self.energy_manager.step_charging()  # Non-blocking step to manage charging
        # Process the adapter states as needed
        for addr, state in adp_states.items():
            if state == AdapterState.IDLE:
                logger.debug(f"Adapter {addr} is idle during charging.")
                continue
            elif state in (AdapterState.REQUESTING, AdapterState.SET_VALUES, AdapterState.CHARGING):
                logger.debug(f"Adapter {addr} is actively charging or requesting. Staying in CHARGING state.")
                return  # Stay in CHARGING state
            elif state == AdapterState.ERROR:  # was AdapterState.FAULT, which does not exist (AttributeError)
                logger.error(f"Adapter {addr} encountered a fault during charging. Transitioning to FAULT state.")
                self.state = State.FAULT
                return
        # If all adapters are idle, transition back to IDLE
        logger.debug("All adapters are idle. Transitioning back to IDLE.")
        self.state = State.IDLE

    async def _handle_low_battery(self) -> None:
        logger.warning("System in LOW_BATTERY state. No charging allowed.")
        # was: ``self.energy_manager.low_battery()`` without await and without any way back to IDLE.
        # low_battery() pauses all adapters while the battery is low; paused adapters resume via
        # REQUESTING once the station is back in IDLE/CHARGING.
        if not await self.energy_manager.low_battery():
            logger.info("Battery recovered. Switching to IDLE.")
            self.state = State.IDLE


    async def _handle_fault(self) -> None:
        logger.error("System in FAULT state. Safety shutdown active.")
        # was: only a sleep - nothing actually switched the outputs off. Repeated every pass on purpose:
        # a failed RS-485 write is retried. (CommManager ends all WiFi sessions on FAULT by itself.)
        #if self.energy_manager is not None:
        #    await self.energy_manager.emergency_off("station FAULT")
        await asyncio.sleep(5.0)

    async def _handle_low_solar_idle(self) -> None:
        logger.warning("System in LOW_SOLAR_IDLE state. No charging allowed.")
        await asyncio.sleep(5.0)

    async def _handle_low_solar_charging(self) -> None:
        logger.warning("System in LOW_SOLAR_CHARGING state. No charging allowed.")
        await asyncio.sleep(5.0)

    async def _handle_debug(self) -> None:
        logger.info("Entering DEBUG state...")
        if self.debug_manager:
            await self.debug_manager.run_cli_session()

        logger.info("Exiting DEBUG State, returning to IDLE.")
        self.state = State.IDLE

    def start_debug_mode(self) -> bool:
        if select.select([sys.stdin], [], [], 0.0)[0]:
            user_cmd = sys.stdin.readline().strip().lower()
            print(user_cmd)
            if user_cmd in ("d", "debug"):
                logger.info("Switching to DEBUG mode triggered by user.")
                self.state = State.DEBUG
                return True
        return False

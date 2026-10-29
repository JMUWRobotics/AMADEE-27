import asyncio
import logging
import select
import sys
from enum import Enum
from pathlib import Path
from typing import Optional

import yaml

from .adapter_manager import AdapterState
from .debug_manager import DebugManager
from .energy_manager import EnergyManager
from .gpio_manager import GPIOManager 
from .types import State   

logger = logging.getLogger("StateMachine")  # Use the module name for logging

BASE_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = BASE_DIR / "config" / "system.yaml"



class StateMachine:
    def __init__(self, config_file: Path = CONFIG_PATH) -> None:
        logger.info("GPIOManager is initializing...")
        self.config_file = config_file
        self.config = None
        self.energy_manager: Optional[EnergyManager] = None
        self.debug_manager: Optional[DebugManager] = None
        self.gpio_manager: Optional[GPIOManager] = None
        self.state = State.BOOT
        self._running = True

    async def run(self) -> None:
        """Main non-blocking async loop executing state handlers and transitions."""
        logger.info("Starting State Machine...")
        if self.state == State.BOOT:
            await self._handle_boot()   
        while self._running:
            if self.gpio_manager is None or self.energy_manager is None:
                logger.error("GPIOManager or EnergyManager not initialized. Exiting loop.")
                break
            await asyncio.sleep(1.0)
            await asyncio.create_task(self.gpio_manager.set_light(self.state))  # Update light state based on current state
            if self.energy_manager.low_battery():
                self.state = State.LOW_BATTERY  
            try:
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

    async def _handle_boot(self) -> None:
        logger.info("Executing BOOT sequence...")
        try:
            with open(self.config_file, "r", encoding="utf-8") as f:
                self.config = yaml.safe_load(f)
            
            if self.config is None:
                logger.error("Configuration file is empty or invalid. Transitioning to FAULT state.")
                self.state = State.FAULT
                return
            else:
                logger.info("Configuration loaded successfully.")
      
            #self.debug_manager = DebugManager(charging_manager=self.charging_manager)
            self.gpio_manager = GPIOManager(config=self.config)
            if self.gpio_manager is None:
                logger.error("GPIOManager failed to initialize. Transitioning to FAULT state.")
                self.state = State.FAULT
                return
            else:
                logger.info("GPIOManager initialized successfully.")

            power_selector_state = self.gpio_manager.read_power_selector_state()
            logger.info(f"Power selector state: {power_selector_state}")

            enable_switches = self.gpio_manager.read_switches()
            logger.info(f"Enable switches state: {enable_switches}")
            self.energy_manager = EnergyManager(config=self.config,power_selector_state=power_selector_state,enable_switches=enable_switches)  # Initial read to cache the state
            if self.energy_manager is None:
                logger.error("EnergyManager failed to initialize. Transitioning to FAULT state.")
                self.state = State.FAULT
                return
            else:
                logger.info("EnergyManager initialized successfully.")
            station_ready = await self.energy_manager.setup_system()

            if station_ready:
                logger.info("Boot sequence completed successfully. Switching to IDLE.")
                self.state = State.IDLE
            else:
                logger.error("Station setup failed during boot.")
                self.state = State.FAULT
                
        except Exception as e:
            logger.error(f"Boot failure: {e}")
            self.state = State.FAULT

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
            elif state == AdapterState.FAULT:
                logger.error(f"Adapter {addr} encountered a fault during charging. Transitioning to FAULT state.")
                self.state = State.FAULT
                return
        # If all adapters are idle, transition back to IDLE
        logger.debug("All adapters are idle. Transitioning back to IDLE.")
        self.state = State.IDLE

    async def _handle_low_battery(self) -> None:
        logger.warning("System in LOW_BATTERY state. No charging allowed.")
        self.energy_manager.low_battery()


    async def _handle_fault(self) -> None:
        logger.error("System in FAULT state. Safety shutdown active.")
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


async def main():
    # Initialize the state machine (uses default config path unless specified)
    sm = StateMachine()

    # Execute the asynchronous loop
    await sm.run()


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s [%(levelname)s] [%(name)s] %(message)s", force=True)
    logging.getLogger("raspcode").setLevel(logging.DEBUG)
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("Program interrupted by user. Shutting down...")





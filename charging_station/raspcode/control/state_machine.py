import asyncio
import logging
from enum import Enum
from pathlib import Path
from typing import Optional

import yaml

from charging_manager import ChargingManager
from vedirectManager import VedirectManager

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("StateMachine")

BASE_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = BASE_DIR / "config" / "system.yaml"

class State(Enum):
    BOOT = 1 # Setup state, the station is booting up, initializing and testing all components
    IDLE = 2 # Idle state, the station is ready to accept charging requests
    CHARGING = 3 # Charging state, the station is currently charging a robot
    LOW_SOLAR = 4 # Low solar state, no robot is allowed to charge with >48V 
    LOW_BATTERY = 5 # Low battery state, no robot is allowed to charge, but the station is still operational
    FAULT = 6 # Fault state, an error has occurred and the station is not operational
    DEBUG = 7 # Debug state, for manually setting parameters

class StateMachine:
    def __init__(self, config_file: Path = CONFIG_PATH) -> None:
        self.config_file = config_file
        self.config = None
        self.charging_manager: Optional[ChargingManager] = None
        self.vedirect_manager: Optional[VedirectManager] = None
        self.state = State.BOOT
        self._running = True

    async def run(self) -> None:
        """Main non-blocking async loop executing state handlers and transitions."""
        logger.info("Starting State Machine...")

        while self._running:
            try:
                if self.state == State.BOOT:
                    await self._handle_boot()

                elif self.state == State.IDLE:
                    await self._handle_operational()

                elif self.state == State.FAULT:
                    await self._handle_fault()

                elif self.state == State.DEBUG:
                    await self._handle_debug()

                await asyncio.sleep(1.0)

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

            self.charging_manager = ChargingManager(config=self.config)
            self.vedirect_manager = VedirectManager(config=self.config)

            charging_manager_ready = await self.charging_manager.setup_system()
            asyncio.create_task(self.vedirect_manager.start())

            if charging_manager_ready:
                logger.info("Boot sequence completed successfully. Switching to IDLE.")
                self.state = State.IDLE
            else:
                logger.error("Charging Manager setup failed during boot.")
                self.state = State.FAULT
        except Exception as e:
            logger.error(f"Boot failure: {e}")
            self.state = State.FAULT

    async def _handle_operational(self) -> None:
        # 1. Update hardware device states
        await self.charging_manager.update_all_states()

        # 2. Get telemetry from MPPT / VE.Direct
        mppt_state = self.vedirect_manager.get_latest_state()

        if mppt_state is not None:
            voltage = getattr(mppt_state, "voltage_bus", 0.0)
            solar_power = getattr(mppt_state, "power_panel", 0.0)

            logger.debug(
                f"[MPPT] Voltage: {voltage:.2f}V | Solar: {solar_power:.1f}W | "
                f"Mode: {getattr(mppt_state.state_of_operation, 'name', 'UNKNOWN')}"
            )

            # 3. Evaluate state transitions
            total_power = sum(self.charging_manager.allocated_power.values())

        # 4. Render UI terminal display
        self.charging_manager.render_ui()

    async def _handle_fault(self) -> None:
        logger.error("System in FAULT state. Safety shutdown active.")
        if self.charging_manager:
            for addr in list(self.charging_manager.devices.keys()):
                await self.charging_manager.disable_output(addr)
        await asyncio.sleep(5.0)

    async def _handle_debug(self) -> None:
        logger.info("System in DEBUG state.")
        await asyncio.sleep(1.0)


async def main():
    # Initialize the state machine (uses default config path unless specified)
    sm = StateMachine()

    # Execute the asynchronous loop
    await sm.run()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("Program interrupted by user. Shutting down...")





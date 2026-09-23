import asyncio
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional
import yaml

from drivers.dpm86xx import (
    BaudRate,
    DPM86XXConfig,
    DPM86XXDevice,
    Protocol,
    SaveSlot,
    display_device_list,
)

# Setup logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("ChargingManager")

BASE_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = BASE_DIR / "config" / "system.yaml"


@dataclass
class DPM86XX_Device:
    device: DPM86XXDevice
    boosted: bool
    name: str
    address: int


class ChargingManager:
    def __init__(self, config: dict):
        """
        Asynchronous manager for DPM86XX buck converters sharing a single serial bus.

        :param devices: List of configured DPM86XX_Device wrappers.
        """
        self.devices_config = self.load_system_config(config)

        self.devices: Dict[int, DPM86XX_Device] = {dev.address: dev for dev in self.devices_config}


        # Protect serial bus from concurrent writes across devices
        self._serial_lock = asyncio.Lock()

        # Track allocated power (in Watts) per device address
        self.allocated_power: Dict[int, float] = {dev.address: 0.0 for dev in self.devices_config}

    async def _execute_serial(self, func, *args, **kwargs):
        """Executes blocking serial I/O in a background thread with lock protection."""
        async with self._serial_lock:
            return await asyncio.to_thread(func, *args, **kwargs)

    async def setup_device(self, address: int, new_id: Optional[int] = None) -> bool:
        """
        Initializes default safe state (Output OFF, Default Power-On OFF, Fast Discharge OFF, 0V/0A).
        """
        if address not in self.devices:
            logger.error(f"Cannot setup device: Address {address} not found.")
            return False

        dev = self.devices[address]

        def _sync_setup():
            with dev.device.api as api:
                api.set_output(False)
                api.set_power_on_default(False)
                api.set_fast_discharge(False)
                api.set_voltage_and_current(0, 0)

                if new_id is not None:
                    api.set_slave_address(new_id)

                api.save_settings(SaveSlot.M0)

        try:
            await self._execute_serial(_sync_setup)
            logger.info(f"Device at address {address} initialized to safe default state.")
            return True
        except Exception as e:
            logger.error(f"Failed to setup device at address {address}: {e}")
            return False

    async def setup_system(self) -> bool:
        for dev in self.devices_config:
            if not await self.setup_device(dev.address):
                return False
        return True

    async def request_output(self, address: int, target_voltage: float, target_power: float) -> bool:
        """
        Requests specific voltage and power for a channel within hardware and system constraints.

        :param address: Device Modbus/Simple slave address.
        :param target_voltage: Target voltage in Volts.
        :param target_power: Target power in Watts.
        """
        if address not in self.devices:
            logger.error(f"Device request rejected: Unknown address {address}.")
            return False

        dev_wrapper = self.devices[address]
        # TODO: add (boosted) bus voltage in config.yaml
        # 1. Hardware Limit Check (Boosted = 65V, Non-Boosted = 50V)
        max_voltage = 65.0 if dev_wrapper.boosted else 50.0
        if target_voltage <= 0 or target_voltage > max_voltage:
            logger.warning(
                f"Device {address} rejected: Requested {target_voltage:.2f}V exceeds limit ({max_voltage}V)."
            )
            return False

        if target_power < 0:
            logger.warning(f"Device {address} rejected: Invalid power request ({target_power}W).")
            return False

        # 2. Calculate target current (I = P / V)
        target_current = target_power / target_voltage if target_voltage > 0 else 0.0

        # 3. Dispatch commands to serial port
        def _sync_apply():
            with dev_wrapper.device.api as api:
                api.set_voltage_and_current(target_voltage, target_current)
                api.set_output(True)

        try:
            await self._execute_serial(_sync_apply)
            self.allocated_power[address] = target_power
            logger.info(
                f"Device {address} enabled: {target_voltage:.2f}V @ {target_current:.3f}A ({target_power:.1f}W)"
            )
            return True
        except Exception as e:
            logger.error(f"Communication error applying output to device {address}: {e}")
            return False

    async def disable_output(self, address: int) -> bool:
        """Disables output on a device and releases its allocated system power."""
        if address not in self.devices:
            return False

        dev_wrapper = self.devices[address]

        def _sync_disable():
            with dev_wrapper.device.api as api:
                api.set_output(False)

        try:
            await self._execute_serial(_sync_disable)
            self.allocated_power[address] = 0.0
            logger.info(f"Device {address} output disabled.")
            return True
        except Exception as e:
            logger.error(f"Failed to disable device {address}: {e}")
            return False

    async def update_all_states(self) -> None:
        """Polls registers for all devices and updates cached states sequentially."""
        for address, dev_wrapper in self.devices.items():
            try:
                await self._execute_serial(dev_wrapper.device.update_state)
            except Exception as e:
                logger.error(f"Failed to fetch state for device {address}: {e}")

    def render_ui(self) -> None:
        """Prints terminal display based on cached device states without blocking serial I/O."""
        print("\x1b[2J\x1b[H", end="")  # Clear screen and reset cursor
        device_list = [dev.device for dev in self.devices.values()]
        display_device_list(device_list)


# ---------------------------------------------------------------------------
# Configuration Loader
# ---------------------------------------------------------------------------

    def load_system_config(self, config: dict) -> List[DPM86XX_Device]:
        dpm_config = config["dpm8624"]
        tty_port = dpm_config["tty_port"]
        baudrate = BaudRate(dpm_config["baudrate"])
        stale_time = dpm_config["stale_time"]

        devices: List[DPM86XX_Device] = []
        for dev_info in dpm_config["bucks"]:
            addr = dev_info["address"]
            cfg = DPM86XXConfig(
                port=tty_port,
                baud_rate=baudrate,
                address=addr,
                protocol=Protocol.SIMPLE,
                stale_time=stale_time,
            )
            dev_obj = DPM86XXDevice(cfg, name=f"DPM8624-{addr}")

            devices.append(
                DPM86XX_Device(
                    device=dev_obj,
                    boosted=dev_info["boosted"],
                    name=f"DPM8624-{addr}",
                    address=addr,
                )
            )

        return devices

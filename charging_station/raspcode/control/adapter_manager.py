import asyncio
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional
import yaml
from mfrc522 import MFRC522

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
class RequestedValues:
    robot_id: int
    tag_id: int
    voltage_mv: float
    current_ma: float
    cutoff_ma: float
    battery_type: int

    def to_string(self):
        return (
            "================ ROBO-TAG GELESEN ================\n"
            f"Robot ID:        #{self.robot_id}\n"
            f"Tag ID (Position): #{self.tag_id}\n"
            f"Voltage:      {self.voltage_mv} V\n"
            f"Max. Current:    {self.current_ma} A\n"
            f"Cutoff Current:     {self.cutoff_ma} A\n"
            "==================================================\n"
        )

class Adapter:
    _dpm_serial_lock = asyncio.Lock()
    _mfrc_serial_lock = asyncio.Lock()

    def __init__(self, dpm_tty_port, dpm_baud_rate, dpm_address: int, dpm_stale_time, mfrc_spi_bus :int,
                 mfrc_chip_select: int, mfrc_baud_rate: int, mfrc_pin_rst, boosted: bool):
        self.dpm_address = dpm_address
        self.boosted = boosted
        dpm_cfg = DPM86XXConfig(
            port=dpm_tty_port,
            baud_rate=dpm_baud_rate,
            address=dpm_address,
            protocol=Protocol.SIMPLE,
            stale_time=dpm_stale_time,
        )
        self.dpm_device: DPM86XXDevice = DPM86XXDevice(dpm_cfg, name=f"DPM8624-{dpm_address}")
        self.mfrc_device = mfrc_chip_select
        self.mfrc: MFRC522 = MFRC522(bus=mfrc_spi_bus, device=mfrc_chip_select, spd=mfrc_baud_rate, pin_rst=mfrc_pin_rst)

    async def _execute_serial(self, func, serial_lock: asyncio.Lock, *args, **kwargs):
        """Executes blocking serial I/O in a background thread with lock protection."""
        async with serial_lock:
            return await asyncio.to_thread(func, *args, **kwargs)

    async def setup_device(self, new_id: Optional[int] = None) -> bool:
        """
        Initializes default safe state (Output OFF, Default Power-On OFF, Fast Discharge OFF, 0V/0A).
        """

        def _dpm_sync_setup():
            with self.dpm_device.api as api:
                api.set_output(False)
                api.set_power_on_default(False)
                api.set_fast_discharge(False)
                api.set_voltage_and_current(0, 0)

                if new_id is not None:
                    api.set_slave_address(new_id)

                api.save_settings(SaveSlot.M0)

        def _mfrc_test_setup():
            v = self.mrfc.Read_MFRC522(self.mfrc.VersionReg)
            if v not in (0x91, 0x92):
                raise Exception("RFID not responding: Device" + str(self.mfrc_device))

        try:
            await self._execute_serial(_dpm_sync_setup, serial_lock=self._dpm_serial_lock)
            await self._execute_serial(_mfrc_test_setup, serial_lock=self._mfrc_serial_lock)
            logger.info(f"Device at address {self.dpm_address} initialized to safe default state.")
            return True
        except Exception as e:
            logger.error(f"Failed to setup device at address {self.dpm_address}: {e}")
            return False

    async def set_output(self, state: bool) -> bool:
        """Disables output on a device and releases its allocated system power."""

        def _sync_disable():
            with self.dpm_device.api as api:
                api.set_output(state)

        try:
            await self._execute_serial(_sync_disable, self._dpm_serial_lock)
            logger.info(f"Device {self.dpm_address} output disabled.")
            return True
        except Exception as e:
            logger.error(f"Failed to disable device {self.dpm_address}: {e}")
            return False

    async def update_state(self):
        """Polls registers for all devices and updates cached states sequentially."""
        try:
            await self._execute_serial(self.dpm_device.update_state, self._dpm_serial_lock)
        except Exception as e:
            logger.error(f"Failed to fetch state for device {self.dpm_address}: {e}"

    async def read_rfid(self):

        def _read_rfid():
            (status, TagType) = self.mfrc.MFRC522_Request(self.mfrc.PICC_REQIDL)
            if status != self.mfrc.MI_OK:
                raise Exception(f"Failed to read RFID: {status}")

            (status, uid) = self.mfrc.MFRC522_Anticoll()
            if status != self.mfrc.MI_OK:
                raise Exception(f"Failed to anticoll RFID: {status}")

            self.mfrc.MFRC522_SelectTag(uid)
            data = self.mfrc.MFRC522_Read(4)
            if data and len(data) < 16:
                raise Exception(f"MFRC522 to small data array: {data}")

            data = data[:16]  # Exakt 16 Bytes nehmen
            payload = data[0:15]
            stored_checksum = data[15]
            calculated_checksum = sum(payload) % 256

            if calculated_checksum != stored_checksum:
                raise Exception(f"MFRC522 calculated Checksum ({calculated_checksum}) mismatch Checksum({stored_checksum}: {data}")

            requested_values = RequestedValues(
                robot_id = data[0],
                tag_id = data[1],
                voltage_mv = ((data[2] * 256) + data[3]) * 0.001,
                current_ma = ((data[4] * 256) + data[5]) * 0.001,
                cutoff_ma = ((data[6] * 256) + data[7]) * 0.001,
                battery_type = data[8]
            )

            return requested_values

        try:
            values = await self._execute_serial(_read_rfid(), self._mfrc_serial_lock)
            logger.info(f"Device {self.dpm_address} output disabled.")
            return True
        except Exception as e:
            logger.error(f"Failed to disable device {self.dpm_address}: {e}")
            return False

    async def request_output(self, target_voltage: float, target_power: float):
        """
        Sets specific voltage and power for a channel within hardware and system constraints.
        """

        max_voltage = 65.0 if self.boosted else 50.0
        if target_voltage <= 0 or target_voltage > max_voltage:
            logger.warningS(
                f"Device {self.dpm_address} rejected: Requested {target_voltage:.2f}V exceeds limit ({max_voltage}V)."
            )
            return False

        if target_power < 0:
            logger.warning(f"Device {self.dpm_address} rejected: Invalid power request ({target_power}W).")
            return False

        # 2. Calculate target current (I = P / V)
        target_current = target_power / target_voltage if target_voltage > 0 else 0.0

        # 3. Dispatch commands to serial port
        def _sync_apply():
            with self.dpm_device.api as api:
                api.set_voltage_and_current(target_voltage, target_current)
                api.set_output(True)

        try:
            await self._execute_serial(_sync_apply)
            self.allocated_power[self.dpm_address] = target_power
            logger.info(
                f"Device {self.dpm_address} enabled: {target_voltage:.2f}V @ {target_current:.3f}A ({target_power:.1f}W)"
            )
            return True
        except Exception as e:
            logger.error(f"Communication error applying output to device {self.dpm_address}: {e}")
            return False



class ChargingManager:
    def __init__(self, config: dict):
        """
        Asynchronous manager for DPM86XX buck converters sharing a single serial bus.

        :param devices: List of configured DPM86XX_Device wrappers.
        """
        self.devices_config = self.load_system_config(config)

        self.adapter: List[Adapter] = []


    async def setup_system(self) -> bool:
        """
        Setup all devices that are configured via the yaml
        """
        async with asyncio.TaskGroup() as tg:
            results = [tg.create_task(adp.setup_device()) for adp in self.adapter]
        return all(results)

    def render_ui(self) -> None:
        """Prints terminal display based on cached device states without blocking serial I/O."""
        print("\x1b[2J\x1b[H", end="")  # Clear screen and reset cursor
        device_list = [dev.device for dev in self.devices.values()]
        display_device_list(device_list)


# ---------------------------------------------------------------------------
# Configuration Loader
# ---------------------------------------------------------------------------

    def load_system_config(self, config: dict) -> List[Adapter]:
        dpm_config = config["adapter"]
        tty_port = dpm_config["dpm_tty_port"]
        baudrate = BaudRate(dpm_config["dpm_baudrate"])
        stale_time = dpm_config["dpm_stale_time"]
        mfrc_spi_bus = dpm_config["mfrc_spi_bus"]
        mfrc_baudrate = dpm_config["mfrc_baudrate"]
        mfrc_pin_rst = dpm_config["mfrc_pin_rst"]

        devices: List[Adapter] = []
        for dev_info in dpm_config["devices"]:
            addr = dev_info["dpm_address"]

            cfg = DPM86XXConfig(
                port=tty_port,
                baud_rate=baudrate,
                address=addr,
                protocol=Protocol.SIMPLE,
                stale_time=stale_time,
            )

            devices.append(
                Adapter(
                    dpm_device=DPM86XXDevice(cfg, name=f"DPM8624-{addr}"),
                    nfc_reader=MFRC522(bus=mfrc_spi_bus, device=dev_info["mfrc_chip_select"], spd=mfrc_baudrate, pin_rst=mfrc_pin_rst),
                    boosted=dev_info["boosted"],
                    name=f"DPM8624-{addr}",
                    address=addr,
                )
            )

        return devices

import asyncio
from typing import List

from control.adapter_manager import AdapterManager
from drivers.dpm86xx import *
from charging_station.raspcode.control.vedirect_manager import VedirectManager


def load_system_config(config: dict):
    dpm_config = config["adapter"]
    tty_port = dpm_config["dpm_tty_port"]
    baudrate = BaudRate(dpm_config["dpm_baudrate"])
    stale_time = dpm_config["dpm_stale_time"]
    mfrc_spi_bus = dpm_config["mfrc_spi_bus"]
    mfrc_baudrate = dpm_config["mfrc_baudrate"]
    mfrc_pin_rst = dpm_config["mfrc_pin_rst"]

    adapter: List[AdapterManager] = []
    for dev_info in dpm_config["devices"]:
        adapter.append(
            AdapterManager(
                dpm_tty_port=tty_port,
                dpm_baud_rate=baudrate,
                dpm_address=dev_info["dpm_address"],
                dpm_stale_time=stale_time,
                dpm_max_voltage=dev_info["dpm_max_voltage"],
                mfrc_spi_bus=mfrc_spi_bus,
                mfrc_chip_select=dev_info["mfrc_chip_select"],
                mfrc_baud_rate=mfrc_baudrate,
                mfrc_pin_rst=mfrc_pin_rst,
            )
        )

    mppt_config = config["mppt"]
    ve_device = VedirectManager(
        tty_port=mppt_config["tty_port"],
        timeout=mppt_config["timeout"],
        serial_test=mppt_config["serial_test"],
    )

    return adapter, ve_device


class EnergyManager:
    def __init__(self, config: dict):
        """
        Asynchronous manager for DPM86XX buck converters sharing a single serial bus.

        :param devices: List of configured DPM86XX_Device wrappers.
        """

        (self.adapter, self.mppt) = load_system_config(config)

    async def setup_system(self) -> bool:
        """
        Setup all devices that are configured via the yaml
        """
        async with asyncio.TaskGroup() as tg:
            results = [tg.create_task(adp.setup_device()) for adp in self.adapter]

        asyncio.create_task(self.mppt.start())
        return all(results)

    async def update_states(self) -> None:
        """
        NFC-Reader read and DPM update states and returned 
        """
        #TODO: NFC-Reader read and DPM update states and returned
        async with asyncio.TaskGroup() as tg:
            results = [tg.create_task(adp.update_state()) for adp in self.adapter]
        
        return all(results)

    async def idle_loop(self) -> None:
        """
        Main loop for the EnergyManager, continuously updating device states.
        """
        while True:
            state = await self.update_states()
            await asyncio.sleep(1.0)  # Adjust the sleep duration as needed

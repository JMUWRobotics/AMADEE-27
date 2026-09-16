from drivers.dpm86xx import *
from pathlib import Path
import yaml
from dataclasses import dataclass

BASE_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = BASE_DIR / "config" / "system.yaml"

USE_ANSI_COLORING = True

#TODO: Load device configuration from a config file instead of hardcoding it here

@dataclass
class DPM86XX_Device:
    device: DPM86XXDevice
    boosted: bool
    name: str
    address: int

with open(CONFIG_PATH, "r", encoding="utf-8") as f:
    config = yaml.safe_load(f)

print(f"Loaded configuration from {CONFIG_PATH}: {config}")

dpm_config = config["dpm8624"]

print(f"Loaded DPM8624 configuration: {dpm_config}")

DPM68XX_TTY_PORT   = dpm_config["tty_port"]
DPM68XX_BAUDRATE   = BaudRate(dpm_config["baudrate"])
DPM68XX_STALE_TIME = dpm_config["stale_time"]

DEVICES: List[DPM86XX_Device] = []

for device in dpm_config["bucks"]:
    DEVICES.append(DPM86XX_Device(
        device=DPM86XXDevice(DPM86XXConfig(DPM68XX_TTY_PORT, DPM68XX_BAUDRATE, device["address"], Protocol.SIMPLE, stale_time=DPM68XX_STALE_TIME), f"DPM8624-{device['address']}"),
        boosted=device["boosted"],
        name=f"DPM8624-{device['address']}",
        address=device["address"]
    ))

print(f"Initialized devices: {[device.name for device in DEVICES]}")

def setup_device(device: DPM86XX_Device, new_id: Optional[int]=None) -> DPM86XXDevice:
    with device.device.api as api:
        api.set_output(False)
        api.set_power_on_default(False)
        api.set_fast_discharge(False)
        api.set_voltage_and_current(0, 0)
        
        if new_id is not None:
            api.set_slave_address(new_id)
        
        api.save_settings(SaveSlot.M0)
    
    return DPM86XXDevice(device.config, device.name)

if __name__ == "__main__":
    # new_dev = setup_device(DEVICES[0])
    # new_dev.update_state()
    # new_dev.display_state()
    
    while True:
        print("\x1b[2J\x1b[H", end="")  # Clear screen and move cursor to home position]")
        # display_device_list(DEVICES)
        
        for device in DEVICES:
            try:
                device.update_state()
            except Exception as e:
                print(f"Error updating device {device.name}: {e}")
            device.display_state()
        
        time.sleep(0.1)
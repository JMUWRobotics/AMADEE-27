from apis.dpm86xx import *

USE_ANSI_COLORING = True

DPM68XX_TTY_PORT   = "/dev/ttyUSB0"
DPM68XX_BAUDRATE   = BaudRate.B9600
DPM68XX_STALE_TIME = 1

DEVICES = [
    DPM86XXDevice(DPM86XXConfig(DPM68XX_TTY_PORT, DPM68XX_BAUDRATE, 10, Protocol.SIMPLE, stale_time=DPM68XX_STALE_TIME), "DPM8624-10"),
    DPM86XXDevice(DPM86XXConfig(DPM68XX_TTY_PORT, DPM68XX_BAUDRATE, 11, Protocol.SIMPLE, stale_time=DPM68XX_STALE_TIME), "DPM8624-11"),
    DPM86XXDevice(DPM86XXConfig(DPM68XX_TTY_PORT, DPM68XX_BAUDRATE, 12, Protocol.SIMPLE, stale_time=DPM68XX_STALE_TIME), "DPM8624-12")
]

def setup_device(device: DPM86XXDevice, new_id: Optional[int]=None) -> DPM86XXDevice:
    with device.api as api:
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
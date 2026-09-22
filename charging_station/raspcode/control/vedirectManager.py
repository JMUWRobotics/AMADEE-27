# -*- coding: utf-8 -*-
"""VedirectController für SmartSolar MPPT 150/35."""
import logging
import yaml
import sys
from ve_utils.utype import UType as Ut
from vedirect_m8.ve_controller import VedirectController
from vedirect_m8 import configure_logging
from pathlib import Path
from dataclasses import dataclass

@dataclass
class MPPT_state:
    """Dataclass für den Zustand des MPPT-Ladereglers."""
    voltage_bus: float #Voltage of the bus in [V]
    voltage_panel: float #Voltage of the solar panel in [V]
    power_panel: float #Power of the solar panel in [W]
    current_bus: float #Current of the bus in [A]
    relay_state: bool #State of the relay (Normal operation = off, Battery low voltage = on)
    off_reason: int #Describes the reason why unit is off
    yield_total: float #Total energy yield in [kWh]
    yield_today: float #Energy yield today in [kWh]
    maximum_power_today: float #Maximum power today in [W]
    yield_yesterday: float #Energy yield yesterday in [kWh]
    maximum_power_yesterday: float #Maximum power yesterday in [W]
    error_code: int #Error code
    state_of_operation: int #State of operation 
    firmware_version: str #Firmware version
    product_id: str #Product ID
    serial_number: str #Serial number
    day_sequence: int #Day sequence number
    Tracker_operation_mode: int #Tracker operation mode (0 = Off, 1 = Voltage/Current limited, 2 = MPP Tracker active)


BASE_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = BASE_DIR / "config" / "system.yaml"

logging.basicConfig()
logger = logging.getLogger("vedirect")

def print_data_callback(packet):
    """Callback zur Ausgabe der empfangenen VE.Direct Pakete."""
    logger.info("%s\n", packet)

def mppt_state_callback_function(packet):
    """Callback zur Verarbeitung der empfangenen VE.Direct Pakete und Aktualisierung des MPPT-Zustands."""
    state = MPPT_state(
        voltage_bus=float(packet.get("V", 0.0))/1000.0,  # Convert mV to V
        voltage_panel=float(packet.get("VPV", 0.0))/1000.0,  # Convert mV to V
        power_panel=float(packet.get("PPV", 0.0)),
        current_bus=float(packet.get("I", 0.0))/1000.0,  # Convert mA to A
        relay_state=bool(packet.get("Relay", False)),
        off_reason=int(packet.get("OR", 0),16), # Convert hex string to int
        yield_total=float(packet.get("H19", 0.0))/100.0, # Convert 0.01 kWh to kWh
        yield_today=float(packet.get("H20", 0.0))/100.0 , # Convert 0.01 kWh to kWh
        maximum_power_today=float(packet.get("H21", 0.0)),
        yield_yesterday=float(packet.get("H22", 0.0))/100.0, # Convert 0.01 kWh to kWh
        maximum_power_yesterday=float(packet.get("H23", 0.0)),
        error_code=int(packet.get("ERR", 0)),
        state_of_operation=int(packet.get("CS", 0)),
        firmware_version=packet.get("FW", ""),
        product_id=packet.get("PID", ""),
        serial_number=packet.get("SER#", ""),
        day_sequence=int(packet.get("HSDS", 0)),
        Tracker_operation_mode=int(packet.get("MPPT", 0))
    )
    logger.info("MPPT State: %s", state)



if __name__ == '__main__':
    with open(CONFIG_PATH, "r") as f:
        config = yaml.safe_load(f)
    mppt_config = config["mppt"]
    configure_logging(config.get("debug", False))

    conf = {
        "serial_port": mppt_config["tty_port"],
        "timeout": mppt_config["timeout"],
    }
    logger.info("Configuration: %s", conf)
    ve = VedirectController(
        serial_conf=conf,
        serial_test=mppt_config["serial_test"],
    )

    ve.read_data_callback(mppt_state_callback_function)
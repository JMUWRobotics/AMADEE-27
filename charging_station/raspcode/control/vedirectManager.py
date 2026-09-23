# -*- coding: utf-8 -*-
"""VedirectController for SmartSolar MPPT 150/35."""
import asyncio
import logging
from typing import Optional

import yaml
import sys
from ve_utils.utype import UType as Ut
from vedirect_m8.ve_controller import VedirectController
from vedirect_m8 import configure_logging
from pathlib import Path
from dataclasses import dataclass
from enum import IntEnum, Flag

# =============================================================================
# ENUMS & DATACLASSES
# =============================================================================

class MpptOffReason(Flag):
    NO_INPUT_POWER             = 0x00000001
    SWITCHED_OFF_POWER_SWITCH  = 0x00000002
    SWITCHED_OFF_DEVICE_MODE   = 0x00000004
    REMOTE_INPUT               = 0x00000008
    PROTECTION_ACTIVE          = 0x00000010
    PAYGO                      = 0x00000020
    BMS                        = 0x00000040
    ENGINE_SHUTDOWN_DETECTION  = 0x00000080
    ANALYSING_INPUT_VOLTAGE    = 0x00000100

class MpptOperationState(IntEnum):
    OFF = 0
    FAULT = 2
    BULK = 3
    ABSORPTION = 4
    FLOAT = 5
    EQUALIZE_MANUAL = 7
    STARTING_UP = 245
    AUTO_EQUALIZE_RECONDITION = 247
    EXTERNAL_CONTROL = 252

class MpptError(IntEnum):
    NO_ERROR = 0
    BATTERY_VOLTAGE_TO_HIGH = 2
    CHARGER_TEMPERATURE_TO_HIGH = 17
    CHARGER_OVER_CURRENT = 18
    CHARGER_CURRENT_REVERSED = 19
    BULK_TIME_LIMIT_EXCEEDED = 20
    CURRENT_SENSOR_ISSUE = 21 #(sensor bias/sensor broken)
    TERMINALS_OVERHEATED = 26
    CONVERTER_ISSUE = 28
    INPUT_VOLTAGE_TOO_HIGH = 33 #Solar Panel
    INPUT_CURRENT_TOO_HIGH = 34 #Solar Panel
    INPUT_SHUTDOWN_EXCESSIVE_BATTERY_VOLTAGE = 38
    INPUT_SHUTDOWN_CURRENT_FLOW_DURING_OFF_MODE = 39
    LOST_COMMUNICATION_WITH_ONE_OF_DEVICES = 65
    SYNCHRONISED_CHARGING_DEVICE_CONFIGURATION_ISSUE = 66
    BMS_CONNECTION_LOST = 67
    NETWORK_MISCONFIGURED = 68
    FACTORY_CALIBRATION_DATA_LOST = 116
    INVALID_INCOMPATIBLE_FIRMWARE = 117
    USER_SETTINGS_INVALID = 119

class MpptTrackerOperationState(IntEnum):
    OFF = 0
    VOLTAGE_OR_CURRENT_LIMITED = 1
    MPP_TRACKER_ACTIVE = 2

@dataclass
class MpptState:
    """Dataclass for the state of the MPPT."""
    voltage_bus: float #Voltage of the bus in [V]
    voltage_panel: float #Voltage of the solar panel in [V]
    power_panel: float #Power of the solar panel in [W]
    current_bus: float #Current of the bus in [A]
    relay_state: bool #State of the relay (Normal operation = off, Battery low voltage = on)
    off_reason: MpptOffReason #Describes the reason why unit is off
    yield_total: float #Total energy yield in [kWh]
    yield_today: float #Energy yield today in [kWh]
    maximum_power_today: float #Maximum power today in [W]
    yield_yesterday: float #Energy yield yesterday in [kWh]
    maximum_power_yesterday: float #Maximum power yesterday in [W]
    error_code: MpptError #Error code
    state_of_operation: MpptOperationState #State of operation
    firmware_version: str #Firmware version
    product_id: str #Product ID
    serial_number: str #Serial number
    day_sequence: int #Day sequence number
    Tracker_operation_mode: MpptTrackerOperationState #Tracker operation mode

# =============================================================================
# VEDIRECT MANAGER CLASS
# =============================================================================

class VedirectManager:
    def __init__(self, config: dict) -> None:
        self.config = config

        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.queue: asyncio.Queue[MpptState] = asyncio.Queue()

        logging.basicConfig()
        self.logger = logging.getLogger("vedirect")

        configure_logging(self.config.get("debug", False))

        self.mppt_config = self.config["mppt"]
        self.conf = {
            "serial_port": self.mppt_config["tty_port"],
            "timeout": self.mppt_config["timeout"],
        }
        self.logger.info("Configuration: %s", self.conf)
        self.ve = VedirectController(
            serial_conf=self.conf,
            serial_test=self.mppt_config["serial_test"],
        )
        self.latest_state = None

    def _mppt_state_callback_function(self, packet: dict) -> None:
        """Callback for converting data from vedirect-pkg into a MpptState dataclass and storing into a queue."""
        try:
            state = MpptState(
                voltage_bus=float(packet.get("V", 0.0))/1000.0,  # Convert mV to V
                voltage_panel=float(packet.get("VPV", 0.0))/1000.0,  # Convert mV to V
                power_panel=float(packet.get("PPV", 0.0)),
                current_bus=float(packet.get("I", 0.0))/1000.0,  # Convert mA to A
                relay_state=True if packet.get("Relay") == "ON" else False,
                off_reason=MpptOffReason(int(packet.get("OR", "0"),16)), # Convert hex string to int
                yield_total=float(packet.get("H19", 0.0))/100.0, # Convert 0.01 kWh to kWh
                yield_today=float(packet.get("H20", 0.0))/100.0 , # Convert 0.01 kWh to kWh
                maximum_power_today=float(packet.get("H21", 0.0)),
                yield_yesterday=float(packet.get("H22", 0.0))/100.0, # Convert 0.01 kWh to kWh
                maximum_power_yesterday=float(packet.get("H23", 0.0)),
                error_code=MpptError(int(packet.get("ERR", 0))),
                state_of_operation=MpptOperationState(int(packet.get("CS", 0))),
                firmware_version=packet.get("FW", ""),
                product_id=packet.get("PID", ""),
                serial_number=packet.get("SER#", ""),
                day_sequence=int(packet.get("HSDS", 0)),
                Tracker_operation_mode=MpptTrackerOperationState(int(packet.get("MPPT", 0)))
            )

            if self.loop:
                self.loop.call_soon_threadsafe(self.queue.put_nowait, state)

            self.logger.info("MPPT State: %s", state)

        except Exception as e:
            self.logger.error("Error in VedirectManager._mppt_state_callback_function: %s", e)


    def _run_worker(self) -> None:
        """Synchronous Worker that runs in a thread."""
        self.logger.info("Start MPPT_Readout in the Background Thread...")
        self.ve.read_data_callback(self._mppt_state_callback_function)

    async def start(self) -> None:
        """Starts the readout of the MPPT as asynchronous Task."""
        self.loop = asyncio.get_running_loop()
        await asyncio.to_thread(self._run_worker)

    async def get_state(self) -> MpptState:
        """Interface for extern moduls."""
        return await self.queue.get()

    def get_latest_state(self) -> Optional[MpptState]:
        """Non-blocking getter for the most recently received state."""
        while not self.queue.empty():
            try:
                self.latest_state = self.queue.get_nowait()
            except asyncio.QueueEmpty:
                break
        return self.latest_state

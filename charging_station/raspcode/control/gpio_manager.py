import asyncio

import RPi.GPIO as GPIO
from typing import Dict
from enum import IntEnum
import logging
from state_machine import State
logger = logging.getLogger(__name__)
light_state_dict: Dict[State, tuple] = {
    State.BOOT: (1, 1, 0),  # Green,Yellow continuous
    State.IDLE: (1, 0, 0),  # Green continuous
    State.CHARGING: (1, 2, 0),  # Green continuous, Yellow flashing 
    State.FAULT: (0, 0, 1),  # Red continuous
    State.DEBUG: (1, 1, 1),  # Green + Yellow continuous
    State.LOW_BATTERY: (0, 2, 0),  # Yellow flashing
    State.LOW_SOLAR_IDLE: (0, 1, 0),  # Yellow continuous
    State.LOW_SOLAR_CHARGING: (2, 2, 0),  # Yellow flashing
}   
class PowerSelectorState(IntEnum):
    OFF = 0
    MPPT = 1
    POWER_SUPPLY = 2
    UNDEFINED = 3  # both feedback pins active - wiring fault or relay in transition

class LightState(IntEnum):
    OFF = 0
    CONTINUOUS = 1
    FLASHING = 2    

class GPIOManager:
    def __init__(self, config: dict):
        self.config = config
        dpm_config = config["adapter"]
        devices = dpm_config["devices"]

        self.power_supply_pin = int(config.get("power_supply_pin", 14))
        self.mppt_pin = int(config.get("mppt_pin", 15))
        self.green_continoues = int(config.get("green_continoues", 23))
        self.green_flashing = int(config.get("green_flashing", 22))
        self.red_continoues = int(config.get("red_continoues", 6))
        self.red_flashing = int(config.get("red_flashing", 5))
        self.yellow_continoues = int(config.get("yellow_continoues", 0))
        self.yellow_flashing = int(config.get("yellow_flashing", 1))
        self.watchdog_enable_pin = int(config.get("watchdog_enable_pin", 18))
        self.watchdog_ping = int(config.get("watchdog_ping", 27))

        self.switches = [(d["dpm_address"],d["gpio_pin"]) for d in devices]
        self.setup_gpio()

    def setup_gpio(self) -> None:
        # Every library that touches the pins (MFRC522 included) must use the same numbering.
        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BCM)
        elif GPIO.getmode() != GPIO.BCM:
            raise RuntimeError("GPIO already configured with a numbering other than BCM")
        GPIO.setup(self.power_supply_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.mppt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.green_continoues, GPIO.OUT)
        GPIO.setup(self.green_flashing, GPIO.OUT)
        GPIO.setup(self.red_continoues, GPIO.OUT)
        GPIO.setup(self.red_flashing, GPIO.OUT)
        GPIO.setup(self.yellow_continoues, GPIO.OUT)
        GPIO.setup(self.yellow_flashing, GPIO.OUT)
        for addr, pin in self.switches:
            GPIO.setup(pin, GPIO.IN)
        GPIO.setup(self.watchdog_enable_pin, GPIO.OUT)
        GPIO.setup(self.watchdog_ping, GPIO.OUT)

    # ------------------------------------------------------------------ power selector

    def read_power_selector_state(self) -> PowerSelectorState:
        """Read the (active-low) feedback pins of the source selector and cache the result."""
        mppt_active = GPIO.input(self.mppt_pin) == GPIO.LOW
        supply_active = GPIO.input(self.power_supply_pin) == GPIO.LOW

        if mppt_active and not supply_active:
            state = PowerSelectorState.MPPT
        elif supply_active and not mppt_active:
            state = PowerSelectorState.POWER_SUPPLY
        else:
            state = PowerSelectorState.UNDEFINED

        if state == PowerSelectorState.UNDEFINED:
            raise RuntimeError("Power selector is in an undefined state")
        
        return state

    def set_light(self, state: State) -> None:
        """Set the light state based on the provided State enum."""
        light_state = light_state_dict.get(state, (LightState.OFF, LightState.OFF, LightState.OFF))
        self.set_light(light_state)

    def set_light(self, state: tuple[LightState, LightState, LightState]) -> None:
        """Set the light state based on the provided LightState enum. State is a tuple of (green_state, yellow_state, red_state)."""
        green_state,yellow_state,red_state, = state

        # Set green light
        GPIO.output(self.green_continoues, GPIO.HIGH if green_state != LightState.OFF else GPIO.LOW)
        GPIO.output(self.green_flashing, GPIO.HIGH if green_state == LightState.FLASHING else GPIO.LOW)

        # Set red light
        GPIO.output(self.red_continoues, GPIO.HIGH if red_state != LightState.OFF else GPIO.LOW)
        GPIO.output(self.red_flashing, GPIO.HIGH if red_state == LightState.FLASHING else GPIO.LOW)

        # Set yellow light
        GPIO.output(self.yellow_continoues, GPIO.HIGH if yellow_state != LightState.OFF else GPIO.LOW)
        GPIO.output(self.yellow_flashing, GPIO.HIGH if yellow_state == LightState.FLASHING else GPIO.LOW)

    def enable_watchdog(self) -> None:
        """Enable the watchdog by setting the enable pin high."""
        GPIO.output(self.watchdog_enable_pin, GPIO.HIGH)  
        asyncio.sleep(0.1)  # Short delay to ensure the watchdog is enabled
        GPIO.output(self.watchdog_enable_pin, GPIO.LOW)

    def read_switches(self) -> Dict[int, bool]:
        """Read the state of all switches and return a dictionary mapping address to state (True for closed, False for open)."""
        switch_states = {}
        for addr, pin in self.switches:
            switch_states[addr] = GPIO.input(pin) == GPIO.LOW  # Assuming active-low switches
        return switch_states
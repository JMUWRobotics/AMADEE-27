from enum import Enum

class State(Enum):
    BOOT = 1 # Setup state, the station is booting up, initializing and testing all components
    IDLE = 2 # Idle state, the station is ready to accept charging requests
    CHARGING = 3 # Charging state, the station is currently charging a robot
    LOW_BATTERY = 4 # Low battery state, no robot is allowed to charge, but the station is still operational
    LOW_SOLAR_IDLE = 5 # Low solar state, no robot is allowed to charge, but the station is still operational    
    LOW_SOLAR_CHARGING = 6 # Low solar state, no robot is allowed to charge, but the station is still operational
    FAULT = 7 # Fault state, an error has occurred and the station is not operational
    DEBUG = 8 # Debug state, for manually setting parameters
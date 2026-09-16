from enum import Enum

class State(Enum):
    BOOT = 1 # Setup state, the station is booting up, initializing and testing all components
    IDLE = 2 # Idle state, the station is ready to accept charging requests
    CHARGING = 3 # Charging state, the station is currently charging a robot
    LOW_SOLAR = 4 # Low solar state, no robot is allowed to charge with >48V 
    LOW_BATTERY = 5 # Low battery state, no robot is allowed to charge, but the station is still operational
    FAULT = 6 # Fault state, an error has occurred and the station is not operational

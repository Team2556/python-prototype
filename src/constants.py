from enum import (IntEnum, auto)
import math
from phoenix6.units import *

#region RoboRio Constants
# included to help with communication and readability
class Rio_DIO(IntEnum):
    ZERRO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    # TODO figure out how CAN works..
    TEN = auto()
    ELEVEN = auto()
    TWELVE = auto()
    THIRTEEN = auto()
    FOURTEEN = auto()
    FIFTEEN = auto()
    SIXTEEN = auto()
    SEVENTEEN = auto()

class Rio_Pnue(IntEnum):
    ZERRO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()


class Rio_PWM(IntEnum):
    ONE = 0
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()

class Rio_Relay(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()

class Rio_Analog(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
#endregion

#region CAN Constants
class CAN_Address(IntEnum):
    ZERRO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()
    ELEVEN = auto()
    TWELVE = auto()
#endregion

#region Elevator Constants

class ElevatorConstants(IntEnum):
        kLeftMotorPort = 0
        kRightMotorPort = 1
        kJoystickPort = 0

        kElevatorKp = 5.0
        kElevatorGearing = 10.0
        kElevatorDrumRadius = 0.0508  # 2 inches in meters
        kCarriageMass = 4

        kMinElevatorHeight = 0.0508  # 2 inches
        kMaxElevatorHeight = 1.27  # 50 inches

        kMaxVelocityMetersPerSecond = 1.5
        kMaxAccelerationMetersPerSecSquared = 0.5


        kSVolts = 0
        kGVolts = 0
        kVVoltSecondPerMeter = 1.5
        kAVoltSecondSquaredPerMeter = 0.75

        kElevatorOffsetMeters = 0
#endregion
class Override_DriveConstant:
    ...
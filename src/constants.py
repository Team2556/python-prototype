import numpy
import robotpy_apriltag as apriltag
from enum import (IntEnum, auto)
import math
from phoenix6.units import *
from wpimath.units import degrees, radians, degreesToRadians, radiansToDegrees, inchesToMeters, inches
from wpimath.trajectory import TrapezoidProfile
from wpilib import SmartDashboard


VISION_DES_ANGLE_deg = 25
VISION_TURN_kP = 0
VISION_STRAFE_kP = 26
CAM_MOUNT_HEIGHT = 12
CAM_MOUNT_PITCH = 25


AprilTagField=apriltag.AprilTagFieldLayout.loadField(apriltag.AprilTagField(3))
AprilTags= apriltag.AprilTagFieldLayout.loadField(apriltag.AprilTagField(3)).getTags() #recommend using this instead of the class below
class AprilTags_height:
    def tag_heights(): #height of apriltags by order of number, in centimeters
        heights = numpy.array([135, 135, 117, 178, 178, 17, 17, 17, 17, 17, 17, 135, 135, 117, 178, 178, 17, 17, 17, 17, 17, 17])
        heights = numpy.array([numpy.nan]+[tag.pose.Z for tag in AprilTags]) #put nan at front so index from 1 works better
        return heights
    ''' ID X Y Z Z-Rotation Y-Rotation (in inches)
 1 657.37 25.80 58.50 126 0
 2 657.37 291.20 58.50 234 0
 3 455.15 317.15 51.25 270 0
 4 365.20 241.64 73.54 0 30
 5 365.20 75.39 73.54 0 30
 6 530.49 130.17 12.13 300 0
 7 546.87 158.50 12.13 0 0
 8 530.49 186.83 12.13 60 0
 9 497.77 186.83 12.13 120 0
 10 481.39 158.50 12.13 180 0
 11 497.77 130.17 12.13 240 0
 12 33.51 25.80 58.50 54 0
 13 33.51 291.20 58.50 306 0
 14 325.68 241.64 73.54 180 30
 15 325.68 75.39 73.54 180 30
 16 235.73-0.15 51.25 90 0
 17 160.39 130.17 12.13 240 0
 18 144.00 158.50 12.13 180 0
 19 160.39 186.83 12.13 120 0
 20 193.10 186.83 12.13 60 0
 21 209.49 158.50 12.13 0 0
 22 193.10 130.17 12.13 300 0'''

#region RoboRio Constants
# included to help with communication and readability
class Rio_DIO(IntEnum):
    ZERO = 0
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
    ZERO = 0
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
    ZERO = 0
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
    THIRTEEN = auto()
    FOURTEEN = auto()
    FIFTEEN = auto()
    SIXTEEN = auto()
    SEVENTEEN = auto()
    EIGHTEEN = auto()
    NINETEEN = auto()
    TWENTY = auto()
#endregion

#region Elevator Constants

class ElevatorConstants():
        kLeftMotorPort = CAN_Address.FOURTEEN
        kRightMotorPort = CAN_Address.FIFTEEN
        kJoystickPort = 0
        kpeak_forward_torque_current = 8 #120
        kpeak_reverse_torque_current = -8 #-120

        kElevatorKp = 2.0
        kElevatorKi = 0.0
        kElevatorKd = .0
        kElevatorGearing = 6#10.0
        kElevatorDrumRadius = 0.0508  # 2 inches in meters
        kCarriageMass = 4 # 4 kg

        kMinElevatorHeight = 0.0508  # 2 inches
        kMaxElevatorHeight = 1.27  # 50 inches
        kElevatorDistanceMovedAfterContactWithLimitSwitch = 0.02

        kMaxVelocityMetersPerSecond = 1.5
        kMaxAccelerationMetersPerSecSquared = 0.5

        kSVolts = 0
        kGVolts = 0.0

        kVVoltSecondPerMeter = 0#1.5
        kAVoltSecondSquaredPerMeter = 0#0.75

        kElevatorOffsetMeters = 0

        kTopLimitSwitchChannel = 2
        kBottomLimitSwitchChannel = 3


#endregion
class Override_DriveConstant:
    ...

class RobotDimensions:
    WIDTH_w_bumpers = inches(36) # inches inchesToMeters(36)#(26+2*3.25)
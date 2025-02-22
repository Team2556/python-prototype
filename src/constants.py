import numpy
import robotpy_apriltag as apriltag
from enum import (IntEnum, auto)
import math
from phoenix6.units import *
from wpimath.units import degrees, radians, degreesToRadians, radiansToDegrees, inchesToMeters, inches
from wpimath.trajectory import TrapezoidProfile
from wpilib import SmartDashboard
from phoenix6.configs.config_groups import Slot0Configs


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
    TWENTYONE = auto()
    TWENTYTWO = auto()
    TWENTYTHREE = auto()
    TWENTYFOUR = auto()
    TWENTYFIVE = auto()
    TWENTYSIX = auto()
    TWENTYSEVEN = auto()
    TWENTYEIGHT = auto()
    TWENTYNINE = auto()
    #coral group
    THIRTY = auto()
    THIRTYONE = auto()
    THIRTYTWO = auto()
    THIRTYTHREE = auto()
    THIRTYFOUR = auto()
    THIRTYFIVE = auto()
    THIRTYSIX = auto()
    THIRTYSEVEN = auto()
    THIRTYEIGHT = auto()
    THIRTYNINE = auto()
    #Algae group
    FORTY = auto()
    FORTYONE = auto()
    FORTYTWO = auto()
    FORTYTHREE = auto()
    FORTYFOUR = auto()
    FORTYFIVE = auto()
    FORTYSIX = auto()
    FORTYSEVEN = auto()
    FORTYEIGHT = auto()
    FORTYNINE = auto()
    FIFTY = auto()
    #Climb group
    FIFTYONE = auto()

#endregion

#region Elevator Constants

class ElevatorConstants():
        kLeftMotorPort = CAN_Address.FOURTEEN
        kRightMotorPort = CAN_Address.FIFTEEN
        kJoystickPort = 0
        kpeak_forward_torque_current = 35 #120
        kpeak_reverse_torque_current = -35 #-120
        kincrement_m_per_sec_held = .25
        kHomingRate = 1/30 # 1 meter in 30 seconds

        kElevatorKp = 1.0
        kElevatorKi = 0.0
        kElevatorKd = .0
        kElevatorGearing = 6 #10.0
        kElevatorDrumRadius = .035/2   # half of 35mm in meters
        kCarriageMass = 4 # 4 kg

        kMinElevatorHeight = 0.00 #0.0508  # 2 inches
        kMaxElevatorHeight = inchesToMeters(26)  # 50 inches TODO: make this smaller
        kElevatorDistanceMovedAfterContactWithLimitSwitch = 0.00002
        ScaredSafetyFactor = 200
        kCoralLv1 = 0.1/ScaredSafetyFactor #height in meters
        kCoralLv2 = 0.32/ScaredSafetyFactor#556
        kCoralLv3 = 0.5588/ScaredSafetyFactor

        kMaxVelocityMetersPerSecond = 1.5/ScaredSafetyFactor
        kMaxAccelerationMetersPerSecSquared = 0.5/ScaredSafetyFactor

        kSVolts = 0
        kGVolts = 0.01

        kVVoltSecondPerMeter = 0#1.5
        kAVoltSecondSquaredPerMeter = 0#0.75

        kElevatorOffsetMeters = 0 #Used in softlimit minimum

        kBottomLeftLimitSwitchChannel = Rio_DIO.ZERO
        kBottomRightLimitSwitchChannel = Rio_DIO.ONE
        kTopLeftLimitSwitchChannel = Rio_DIO.TWO
        kTopRightLimitSwitchChannel = Rio_DIO.THREE #TODO: ? two on top also?




#endregion
class Override_DriveConstant:
    ...
    
class AlgaeConstants:
    kIntakeCANAddress1 = CAN_Address.FORTY # TODO: Correct the CAN Addresses pls
    kIntakeCANAddress2 = CAN_Address.FORTYONE
    kAlgaeLimitSwitchChannel = Rio_DIO.FOUR # So it doesn't input when limit switch activated

class RobotDimensions:
    WIDTH_w_bumpers = inches(36) # inches inchesToMeters(36)#(26+2*3.25)

class CoralConstants:
    kCoralMotorPort = CAN_Address.THIRTY
    kLeftBreakerLight = 8 # TODO: Get the actual IDs
    kRightBreakerLight = 9

class ClimbConstants:
    kClimbMotorPort = CAN_Address.FIFTYONE
    kTopLimitSwitchChannel = Rio_DIO.FIVE
    kBottomLimitSwitchChannel = Rio_DIO.SIX
    kClimbHookZeroEntry = 1
    kSpeed = 1.0
    kMaxAccelerationMetersPerSecondSquared = 0.5
    #kPClimbController = 1
    #kIClimbController = 0
    #kDClimbController = 0
    kClimbMaxHeight = 1.6081375
    kClimbMinHeight = 0.815975
    # kClimbSlightlyExtendedHeight = 0.9
    # kClimbMinHeightEncoderEstimate = -3583.037109375
    kS = 0
    kV = 0
    kA = 0
    kClimbMaxPosConfirmationExtraHeight = 0.1
    GEAR_RATIO = 15376/45
    kPositionConversionFactor = 0.1965
    GAINS = (Slot0Configs()
                .with_k_p(1.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0.0)
                .with_k_a(0.0)
            )

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


AprilTagField=apriltag.AprilTagFieldLayout.loadField(apriltag.AprilTagField.k2025ReefscapeWelded)

AprilTags= apriltag.AprilTagFieldLayout.loadField(apriltag.AprilTagField.k2025ReefscapeWelded).getTags() #recommend using this instead of the class below
'''AprilTags is a list of all the april tag objects on the field, in order of number'''
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

# region RoboRio Constants
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

#endregion

#region Elevator Constants

class ElevatorConstants():
        # Motor CAN ID's
        kLeftMotorPort = CAN_Address.FOURTEEN
        kRightMotorPort = CAN_Address.FIFTEEN
        # kJoystickPort = 0
        
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
        
        ScaredSafetyFactor = 200 # Set ScaredSafetyFactor to 1 once we get SUPER confident
        # All the elevator levels to do stuff
        kCoralLv1 = 0.1 / ScaredSafetyFactor # Height in meters
        kCoralLv2 = 0.32 / ScaredSafetyFactor # 556
        kCoralLv3 = 0.5588 / ScaredSafetyFactor
        kCoralLv4 = 0.6 / ScaredSafetyFactor # All the elevator levels below aren't tuned
        kAlgaeProcess = 0.15 / ScaredSafetyFactor
        kAlgaeGroundIntake = 0.21 / ScaredSafetyFactor
        kAlgaeLv2 = 0.2 / ScaredSafetyFactor
        kAlgaeLv3 = 0.4 / ScaredSafetyFactor
        kZero = 0

        # The command decides the position's close enough if it's within this range (in meters?)
        kTargetValueAccuracy = 0.02

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

class RobotDimensions:
    WIDTH_w_bumpers = inches(36) # inches inchesToMeters(36)#(26+2*3.25)

#region ALGAE YAY

class AlgaeConstants:
    # Motor Channels
    kPivotMotorChannel = CAN_Address.FORTY # TODO: Add the actual CAN IDs
    kIntakeWheelsChannel = CAN_Address.FORTYONE
    
    # Limit Switch channel (So it doesn't input when limit switch activated)
    kLimitSwitchChannel = Rio_DIO.FOUR # TODO: Add more actual CAN IDs
    # kOtherLimitSwitchChannel = Rio_DIO.FIVE # TODO: Figure out if there's actually two limit switches
    
    # This is so it doesn't move too fast in one way? 
    # (to disable just set to super high positive/negative numbers)
    kPeakForwardTorqueCurrent = 10
    kPeakReverseTorqueCurrent = -10
    
    # All the following stuff are tunable in SmartDashboard
    kPivotp = 0.3
    kPivoti = 0
    kPivotd = 0
    kPivotg = 0
    
    # If the motor is stalled then it's trying to intake an algae more than it can
    # So this detects if it shouldn't spin anymore
    kAmpValueToDetectIfMotorStalled = 90
    
    # The command decides the position's close enough if it's within this range (in rotations)
    kTargetValueAccuracy = 0.01
    
    kPivotMaxHeight = 0.25
    kPivotMinHeight = 0
    
    # Values to set pivoting motor to
    kPivotReefIntakingValue = 0.25 # Pivot position when grabbing algae
    kPivotGroundIntakingValue = 0.03 # Pivot position when grabbing algae from the FLOOR
    kPivotProcessingValue = 0.07 # Pivot position when about to send to processor
    kPivotIdleValue = 0 # Pivot position when idle
    # The time it takes to switch between pivoting positions
    kPivotRotationsPerSecond = 0.05
    
    # Intake wheels multiply by this speed
    kIntakeMultiplier = 0.8 # CHANGE BACK TO 0.2 WHEN TESTING
    
    # The code waits this many seconds between intaking/processing
    kTimeItTakesToIntake = 1
    kTimeItTakesToProcess = 1

# vvv OUTDATED GET RID OF WHEN MERGING vvv
# class CoralConstants:
#     kCoralTrackChannel = CAN_Address.THIRTY
    
#     kLeftSolenoidChannel = 1 # TODO: Get the actual IDs
#     kRightSolenoidChannel = 2
    
#     # Coral track tuning constants
#     kTrackCenterMultiplier = 0.08
#     kTrackDischargeMultiplier = 0.16
#     kCoralDischargeTime = 1.5
    
#     kLeftBreakerLight = Rio_DIO.FIVE # TODO: Get the actual IDs
#     kRightBreakerLight = Rio_DIO.SIX
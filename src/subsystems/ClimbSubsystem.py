import commands2
import wpilib
from wpilib import SmartDashboard
import wpimath.controller
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.units import meters, inches, seconds, metersToInches, inchesToMeters
import phoenix6
from phoenix6 import hardware, controls, configs, StatusCode, signals
from phoenix6.controls import Follower
from phoenix6.signals import NeutralModeValue
from constants import ClimbConstants
from math import pi
import ntcore

class ClimbSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.climbmotor = phoenix6.hardware.TalonFX(ClimbConstants.kClimbMotorPort, "rio")
        self.climbmotor.setNeutralMode(NeutralModeValue.BRAKE)
        self.climbCANcoder = phoenix6.hardware.CANcoder(ClimbConstants.kClimbMotorPort)
        self.limit_bottom = wpilib.DigitalInput(ClimbConstants.kBottomLimitSwitchChannel)
        self.limit_top = wpilib.DigitalInput(ClimbConstants.kTopLimitSwitchChannel)


        self.tableinstance = ntcore.NetworkTableEntry()
        self.basetable = ntcore.NetworkTable()
        self.climbhookzeroentry = ntcore.NetworkTableEntry(ClimbConstants.kClimbHookZeroEntry)
        self.climbhookheightentry = ntcore.NetworkTableEntry()
        self.climbzeroreceivedsuccessfully = ntcore.NetworkTableEntry()

        self.controller = ProfiledPIDController(
            ClimbConstants.kPClimbController,
            ClimbConstants.kIClimbController,
            ClimbConstants.kDClimbController, 
            constraints=TrapezoidProfile.Constraints(ClimbConstants.kMaxVelocityMetersPerSecond, ClimbConstants.kMaxAccelerationMetersPerSecondSquared),
            period=0.02
        )
        #self.feedforward = Feedforward(ks=0.1, kv=0.1)

    




import commands2
import wpilib
from wpilib import SmartDashboard
from phoenix6 import hardware
from enum import Enum, auto
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, MotorOutputConfigs, FeedbackConfigs
from phoenix6.controls import VoltageOut
from wpiutil import SendableBuilder
from constants import ClimbConstants
from wpimath import units
#from subsystems import StateSubsystem


class ClimbSubsystem(commands2.Subsystem):
    class SubsystemState(Enum):
        INITIAL = auto()
        STOP = auto()
        CLIMB_POSITIVE = auto()
        CLIMB_NEGATIVE = auto()
        UNKNOWN = auto()
        RESET = auto()
        
    

    position_conversion_factor = ClimbConstants.kPositionConversionFactor
    speed = ClimbConstants.kSpeed
    _motor_config = (TalonFXConfiguration()
                     .with_slot0(ClimbConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(ClimbConstants.GEAR_RATIO))
                     )
    
    _state_configs: dict[SubsystemState, tuple[int, units.degrees]] = {
        SubsystemState.STOP: (0, 180),
        SubsystemState.CLIMB_POSITIVE: (4, 0),
        SubsystemState.CLIMB_NEGATIVE: (-4, 0),
        SubsystemState.INITIAL: (0,0),
        SubsystemState.RESET: (0,0),
    }
    
    def __init__(self) -> None:
        super().__init__()
        self.climbmotor = hardware.TalonFX(ClimbConstants.kClimbMotorPort, "rio")
        climbmotor_configurator = self.climbmotor.configurator
        self.climbCANcoder = hardware.CANcoder(ClimbConstants.kClimbMotorPort)
        self.limit_bottom = wpilib.DigitalInput(ClimbConstants.kBottomLimitSwitchChannel)
        self.limit_top = wpilib.DigitalInput(ClimbConstants.kTopLimitSwitchChannel)
        self.climb_request = VoltageOut(0)
        climbmotor_configurator.apply(self._motor_config)
        self.isClimbExtended = False
        self.isSmartDashboardTestControlsShown = False
        self.mode = ""  # auto, teleop, test, disabled
        self._offset = 0.0
        


    def set_desired_state(self, desired_state: SubsystemState) -> None:
        # if not super().__setattr__(desired_state):
        #     return
            # self._subsystem_state = desired_state
            # match self._subsystem_state:
            #     case self.SubsystemState.STOP:
            #         self.climb_request.output = 0
            #     case self.SubsystemState.CLIMB_POSITIVE:
            #         self.climb_request.output = 4
            #         self.isClimbExtended = True
            #     case self.SubsystemState.CLIMB_NEGATIVE:
            #         self.climb_request.output = -4
            #         self.isClimbExtended = False


        output = self._state_configs.get(desired_state, (0, 180))
        self.climb_request.output = output
        self.climbmotor.set_control(self.climb_request)

    def getEncoderValue(self):
        return self.climbCANcoder.get_position()
    

    def getIsClimbExtended(self):
        return self.isClimbExtended
    
    def putSmartDashboardControlCommands(self):
        """Puts values on Shuffleboard (via SmartDashboard) for testing"""
        SmartDashboard.putBoolean("ClimbExtended", self.isClimbExtended)

    def setValuesFromSmartDashboard(self):
        """Updates Climb state based on Shuffleboard inputs"""
        lastState = self.isClimbExtended
        requestedState = SmartDashboard.getBoolean("ClimbExtended", False)
        if lastState != requestedState:
            if requestedState:
                self.climbDeploy()
            else:
                self.climbRetract()
            
    def periodic(self) -> None:
        if self.mode == "test":
            if not self.isSmartDashboardTestControlsShown:
                self.putSmartDashboardControlCommands()
            self.setValuesFromSmartDashboard()
            
        self._prev_is_down = self.limit_bottom.get()

        if self._prev_is_down and not self.limit_bottom.get():
            self._offset = (
                ClimbConstants.kClimbMaxHeight / self.position_conversion_factor
                - self.climbCANcoder.get_position()
            )
            self._has_reset = True
        

    def simulationPeriodic(self) -> None:
        distance = self.climbmotor.get()
        self.climbCANcoder.set_position(
            self.climbCANcoder.get_position() + distance / self.position_conversion_factor
        )

        if self.getPosition() >= 90:
            self.limit_top.setSimDevice()
        else:
            self.limit_bottom.setSimDevice()

    def isInitial(self):
        return self.SubsystemState == self.SubsystemState.INITIAL

    def isClimbed(self):
        return self.limit_top.get()
    
    def isReset(self):
        return self.limit_bottom.get()

    def setSpeed(self, speed: float):
        if self.getPosition() <= 0.0:
            self.climbmotor.set(speed if speed >= 0 else 0)
        elif self.isClimbed():
            self.climbmotor.set(speed if speed <= 0 else 0)
        else:
            self.climbmotor.set(speed)

    def stop(self):
        self.climbmotor.stopMotor()

    def getPosition(self):
        return self.position_conversion_factor * (
            self.climbCANcoder.get_position() + self._offset
        )

    def climbDeploy(self, speed):
        self.isClimbExtended = True
        self.climbmotor.setVoltage(speed)

    def climbPull (self, speed):
        self.isClimbExtended = True
        self.climbmotor.setVoltage(speed * 0.5)

    def climbRetract(self, speed):
        self.isClimbExtended = False
        self.climbmotor.setVoltage(-speed)

    def initSendable(self, builder: SendableBuilder) -> None:
        super().initSendable(builder)

        def setOffset(value: float):
            self._offset = value

        def setHasReset(value: bool):
            self._has_reset = value

        # def noop(x):
        #     pass

        SmartDashboard.putStringArray("state", lambda: self.SubsystemState)
        SmartDashboard.putNumber("motor_input", self.climbmotor.get())
        SmartDashboard.putNumber("encoder", self.climbCANcoder.get_position())
        SmartDashboard.putNumber("position", self.getPosition())
        SmartDashboard.putNumber("offset", lambda: self._offset, lambda x: setOffset(x))
        SmartDashboard.putNumber("has_reset", lambda: self._has_reset, setHasReset)




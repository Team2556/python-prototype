import commands2
import wpilib
from wpilib import SmartDashboard, DriverStation
from phoenix6 import hardware
from enum import Enum
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, MotorOutputConfigs, FeedbackConfigs
from constants import ClimbConstants
from subsystems import StateSubsystem


class ClimbSubsystem(StateSubsystem):
    class SubsystemState(Enum):
        STOP = 1
        CLIMB_POSITIVE = 2
        CLIMB_NEGATIVE = 3

    
    _motor_config = (TalonFXConfiguration()
                     .with_slot0(ClimbConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(ClimbConstants.GEAR_RATIO))
                     )
    def __init__(self) -> None:
        super().__init__("Climb")
        climbmotor_configurator = self.climbmotor.configurator
        self.climbmotor = hardware.TalonFX(ClimbConstants.kClimbMotorPort, "rio")
        self.climbCANcoder = hardware.CANcoder(ClimbConstants.kClimbMotorPort)
        self.limit_bottom = wpilib.DigitalInput(ClimbConstants.kBottomLimitSwitchChannel)
        self.limit_top = wpilib.DigitalInput(ClimbConstants.kTopLimitSwitchChannel)
        climbmotor_configurator.apply(self._motor_config)


    def ClimbPeriodic(self):
       # super().ClimbPeriodic("ClimbPeriodic")
        pass

    def set_desired_state(self, desired_state: SubsystemState) -> None:
            self._subsystem_state = desired_state
            match self._subsystem_state:
                case self.SubsystemState.STOP:
                    self.climbCANcoder.set_control = 0
                case self.SubsystemState.CLIMB_POSITIVE:
                    self.climbCANcoder.set_control = 0.5
                case self.SubsystemState.CLIMB_NEGATIVE:
                    self.climbCANcoder.set_control = -0.5

            self.climbmotor.set_control(self.climbCANcoder)
#         self.tableinstance = ntcore.NetworkTableEntry()
#         self.basetable = ntcore.NetworkTable()
#         self.climbhookzeroentry = ntcore.NetworkTableEntry(ClimbConstants.kClimbHookZeroEntry)
#         self.climbhookheightentry = ntcore.NetworkTableEntry()
#         self.climbzeroreceivedsuccessfully = ntcore.NetworkTableEntry()

#      #   self.controller = ProfiledPIDController(
#         #    ClimbConstants.kPClimbController,
#         #    ClimbConstants.kIClimbController,
#         #    ClimbConstants.kDClimbController, 
#         #    constraints=TrapezoidProfile.Constraints(ClimbConstants.kMaxVelocityMetersPerSecond, ClimbConstants.kMaxAccelerationMetersPerSecondSquared),
#         #    period=0.02
#         #)
#         self.ks = ClimbConstants.kS
#         self.kv = ClimbConstants.kV
#         self.ka = ClimbConstants.kA
#         self.feedforward = SimpleMotorFeedforwardMeters(ks, kv, ka)


        
#         climbmotor_configurator = self.climbmotor.configurator
#         cfg = configs.MotorOutputConfigs()  
#         cfg.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        
#         climbmotor_configurator.apply(cfg)


#     # def getClimbHookZeroValue(self):
#     #     return self.ClimbHookZero

#     # def setClimbHookZeroValue(self, value):
#     #     self.ClimbHookZero = value
    def getEncoderValue(self):
        return self.climbCANcoder.get_position()
    
    def getIsBottomLimitSwitchPressed(self):
        return self.limit_bottom.get()
    
    def getIsTopLimitSwitchPressed(self):
        return self.limit_top.get()

#     def getHookHeight(self):
#         multiplier = (ClimbConstants.kClimbMaxHeight - ClimbConstants.kClimbMinHeight) / \
#                      (0 - ClimbConstants.kClimbMinHeightEncoderEstimate)
#         if not self.getIsTopLimitSwitchPressed():
#             return multiplier * self.getEncoderValue() + ClimbConstants.kClimbMaxHeight
#         else:
#             return ClimbConstants.kClimbMinHeight

#     def getHookHeightNoLimits(self):
#         multiplier = (ClimbConstants.kClimbMaxHeight - ClimbConstants.kClimbMinHeight) / \
#                      (0 - ClimbConstants.kClimbMinHeightEncoderEstimate)
#         return multiplier * self.getEncoderValue() + ClimbConstants.kClimbMaxHeight




#   #  def getPIDError(self):
#   #      return self.controller.getPositionError()


#    # def getIsClimbVertical(self):
#     #    if self.climbmotor.get() == wpilib.DoubleSolenoid.Value.kOff:
#      #       DriverStation.reportWarning("The climb motor should never be off. Something is wrong.", True)
#      #   elif self.climbmotor.get() == wpilib.DoubleSolenoid.Value.kForward:
#       #      return True
#       #  elif self.climbmotor.get() == hardware.TalonFX.get_reverse_limit:
#       #      return False
#       #  else:
#       #      DriverStation.reportWarning("For some reason, the climb motor is in an unknown state.", True)
#       #  return True

#     def setMotorOutputManual(self, output):
#         self.climbmotor.set(ClimbConstants.kSpeed, output)

 

#   #  def getMotorOutput(self, ff, desired, current):
#    #     self.controller.calculate(current, desired)
#      #   return ((desired - current) * self.controller.getP()) + ff.calculate(self.controller.getGoal().velocity)

#  #   def setDesiredHookHeight(self, height):
#      #   Output = self.getMotorOutput(self.controller, self.FeedForward, height, self.getHookHeight())
#      #   self.climbmotor.set(controls.VoltageOut, Output)


#    # def setPos(self, height):
#     #    self.setDesiredHookHeight(height)
        

#  #   def settHookToBottomPos(self):
#     #    leftOutput = self.controller.calculate(self.getLeftHookHeight(), ClimbConstants.kClimbMinHeight)
#        # self.setHookToBottomPos(leftOutput, self.m_climbLeftProfiledPIDController.getPositionError(), self.m_leftMotor)


#   #  def setHookToBottomPos(self, outputFromPID, errorFromPID, motor):
#     #    if errorFromPID <= ClimbConstants.kClimbMinPosPIDErrorThreshold and errorFromPID != 0:
#     #        motor.set(ClimbConstants.kClimbVoltageToApplyAfterPID / 12)
#     #    elif errorFromPID > ClimbConstants.kClimbMinPosPIDErrorThreshold:
#      #       motor.set(outputFromPID)
#      #   else:
#      #       motor.set(ClimbConstants.kClimbVoltageToHoldBottomPosition / 12)

#    # def setHookToFullExtension(self):
#    #     self.setDesiredHookHeight(ClimbConstants.kClimbMaxHeight + ClimbConstants.kClimbMaxPosConfirmationExtraHeight)


#  #   def setClimbVertical(self):
#       #  self.m_solenoid.set(wpilib.DoubleSolenoid.Value.kForward)

#    # def setClimbAngled(self):
#    #     self.m_solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

#     def resetEncoders(self):
#         self.climbCANcoder.set_position(0)
        

#     def TeleopPeriodic(self):
#         manual_climb_speed = ClimbConstants.kSpeed * 0.5
#         if self.limit_top.get():
#             # Limit switch is pressed, stop winch if trying to retract
#             if  manual_climb_speed < 0:
#                 self.climbmotor.set(0)
#             else:
#               self.climbmotor(manual_climb_speed)
#         else:
#             # Limit switch is not pressed, control winch with joystick
#             self.climbmotor.set(manual_climb_speed)

#         if self.limit_bottom.get():
#             # Limit switch is pressed, stop winch if trying to retract
#             if  manual_climb_speed < 0:
#                 self.climbmotor.set(0)
#             else:
#               self.climbmotor(manual_climb_speed)
#         else:
#             # Limit switch is not pressed, control winch with joystick
#             self.climbmotor.set(manual_climb_speed)




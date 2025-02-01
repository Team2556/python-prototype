import commands2
import wpilib
import wpimath.controller
import wpimath.trajectory
import phoenix6
from constants import ElevatorConstants
from phoenix6 import configs, signals


class ElevatorSubsystem(commands2.ProfiledPIDSubsystem):

# Create a new ElevatorSubsystem
    def __init__(self) -> None:
        '''IM AN ELEVATOR'''
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                ElevatorConstants.kElevatorKp,
                0,
                0,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    ElevatorConstants.kMaxVelocityMetersPerSecond,
                    ElevatorConstants.kMaxAccelerationMetersPerSecSquared,
                ),
            ),
            0,
        )
        self.elevcontroller = super().getController #wpimath.controller.ProfiledPIDController(5.0, 0, 0))
        self.elevmotor_right = phoenix6.hardware.TalonFX(ElevatorConstants.kRightMotorPort, "rio")
        self.elevmotor_left = phoenix6.hardware.TalonFX( ElevatorConstants.kLeftMotorPort, "rio")
        self.topelevmotorlimitswitch = wpilib.DigitalInput(ElevatorConstants.kTopLimitSwitchChannel)
        self.bottomelevmotorlimitswitch = wpilib.DigitalInput(ElevatorConstants.kBottomLimitSwitchChannel)
        self.joystick2 = commands2.button.CommandXboxController(ElevatorConstants.kJoystickPort)
        self.feedforward = wpimath.controller.ElevatorFeedforward(
            ElevatorConstants.kSVolts,
            ElevatorConstants.kGVolts,
            ElevatorConstants.kVVoltSecondPerMeter,
            ElevatorConstants.kAVoltSecondSquaredPerMeter,
        )
        
        # Start elevator at rest in neutral position
        self.setGoal(ElevatorConstants.kElevatorOffsetMeters)

    def _configs(self) -> None:
        leftmotor_configs = configs.MotorOutputConfigs()
        rightmotor_configs = configs.MotorOutputConfigs()
        leftmotor_configurator = self.elevmotor_left.configurator
        rightmotor_configurator = self.elevmotor_right.configurator
# set invert to CW+ and apply config change
        leftmotor_configs.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        rightmotor_configs.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        rightmotor_configurator.apply(rightmotor_configs)
        leftmotor_configurator.apply(leftmotor_configs)
        # Create current limiter

        limit_configs = configs.CurrentLimitsConfigs()

        limit_configs.stator_current_limit = 120
        limit_configs.stator_current_limit_enable = True

        leftmotor_configurator.apply(limit_configs)
        rightmotor_configurator.apply(limit_configs)

    def _useOutput(
        self, output: float, setpoint: wpimath.trajectory.TrapezoidProfile.State
    ) -> None:
        # Calculate the feedforward from the setpoint
        feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)

        # Add the feedforward to the PID output to get the motor output
        self.elevmotor_right.setVoltage(output + feedforward)
        self.elevmotor_left.setVoltage(output + feedforward)

    def _getMeasurement(self) -> float:
        return self.elevmotor_right.set_position() + ElevatorConstants.kElevatorOffsetMeters
    
    def _getMeasurement (self) -> float:
        return self.elevmotor_left.set_position() + ElevatorConstants.kElevatorOffsetMeters
    
    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""
        self.disable()

    def moveElevator(self, meters = ElevatorConstants.kElevatorOffsetMeters) -> None:
        self.setGoal(meters)
        self.enable()

    def elevatorPeriodic(self) -> None:

        if self.joystick2.povUp():
            # Here, we run PID control like normal, with a constant setpoint of 30in (0.762 meters).
            pidOutput = self.elevcontroller.calculate(self.elevmotor_right.set_position(), 1.27, 
                self.elevmotor_left.set_position(), 1.27)
            self.elevmotor_right.setVoltage(pidOutput)
            self.elevmotor_left.setVoltage(pidOutput)
        elif self.joystick2.povRight():
            pidOutput = self.elevcontroller.calculate(self.elevmotor_right.set_position(), 0.762,
                self.elevmotor_left.set_position(), 0.762)
            self.elevmotor_right.setVoltage(pidOutput)
            self.elevmotor_left.setVoltage(pidOutput)
        elif self.joystick2.povLeft():
            pidOutput = self.elevcontroller.calculate(self.elevmotor_right.set_position(), 0.3, 
                self.elevmotor_left.set_position(), 0.3)
            self.elevmotor_right.setVoltage(pidOutput)
            self.elevmotor_left.setVoltage(pidOutput)
        elif self.joystick2.povDown():
            pidOutput = self.elevcontroller.calculate(self.elevmotor_right.set_position(), 0,
                self.elevmotor_left.set_position(), 0)
            self.elevmotor_right.setVoltage(pidOutput)
            self.elevmotor_left.setVoltage(pidOutput)
        else:
            # Otherwise we disable the motor
            self.elevmotor_right.set(0.0)
            self.elevmotor_left.set(0.0)


        if self.topelevmotorlimitswitch.get():
            self.moveElevator(meters = self.getMeasurement - ElevatorConstants.kElevatorDistanceMovedAfterContactWithLimitSwitch)
        elif self.bottomelevmotorlimitswitch.get():
            self.moveElevator(meters = self.getMeasurement + ElevatorConstants.kElevatorDistanceMovedAfterContactWithLimitSwitch)
            
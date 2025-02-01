import commands2
import wpilib
import wpimath.controller
import wpimath.trajectory
import phoenix6
from constants import ElevatorConstants


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
        self.elevcontroller = super().getController# .controler# wpimath.controller.ProfiledPIDController(5.0, 0, 0)
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

    def _useOutput(
        self, output: float, setpoint: wpimath.trajectory.TrapezoidProfile.State
    ) -> None:
        # Calculate the feedforward from the setpoint
        feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)

        # Add the feedforward to the PID output to get the motor output
        self.elevmotors.setVoltage(output + feedforward)
    def _getMeasurement(self) -> float:
        return self.elevmotors.set_position() + ElevatorConstants.kElevatorOffsetMeters
        
    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""
        self.disable()

    def moveElevator(self, meters = ElevatorConstants.kElevatorOffsetMeters) -> None:
        self.setGoal(meters)
        self.enable()

    def elevatorPeriodic(self) -> None:

        if self.joystick2.rightTrigger():
            # Here, we run PID control like normal, with a constant setpoint of 30in (0.762 meters).
            pidOutput = self.elevcontroller.calculate(self.elevmotors.set_position(), 1.27)
            self.elevmotors.setVoltage(pidOutput)
        elif self.joystick2.leftTrigger():
            pidOutput = self.elevcontroller.calculate(self.elevmotors.set_position(), 0.762)
            self.elevmotors.setVoltage(pidOutput)
        elif self.joystick2.leftBumper():
            pidOutput = self.elevcontroller.calculate(self.elevmotors.set_position(), 0.3)
            self.elevmotors.setVoltage(pidOutput)
        elif self.joystick2.rightBumper():
            pidOutput = self.elevcontroller.calculate(self.elevmotors.set_position(), 0)
            self.elevmotors.setVoltage(pidOutput)
        else:
            # Otherwise we disable the motor
            self.elevmotors.set(0.0)


        if self.topelevmotorlimitswitch.get():
            self.moveElevator(meters = self.getMeasurement - ElevatorConstants.kElevatorDistanceMovedAfterContactWithLimitSwitch)
        elif self.bottomelevmotorlimitswitch.get():
            self.moveElevator(meters = self.getMeasurement + ElevatorConstants.kElevatorDistanceMovedAfterContactWithLimitSwitch)
            
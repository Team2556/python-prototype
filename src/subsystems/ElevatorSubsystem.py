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

        self.rightelevmotor = phoenix6.hardware.TalonFX(ElevatorConstants.kRightMotorPort)
        self.leftelevmotor = phoenix6.hardware.TalonFX(ElevatorConstants.kLeftMotorPort)
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
        self.rightelevmotor.setVoltage(output + feedforward)
        self.leftelevmotor.setVoltage(output + feedforward)
    def _getMeasurement(self) -> float:
        return self.rightelevmotor.set_position() + ElevatorConstants.kElevatorOffsetMeters
    
    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""
        self.disable()

    def moveElevator(self, meters = ElevatorConstants.kElevatorOffsetMeters) -> None:
        self.setGoal(meters)
        self.enable()

import commands2
import wpilib
import wpimath.controller
import wpimath.trajectory
import phoenix6

import constants

class ElevatorSubsystem(commands2.ProfiledPIDSubsystem):


# Create a new ElevatorSubsystem
    def __init__(self) -> None:
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                constants.ElevatorConstants.kElevatorKp,
                0,
                0,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    constants.ElevatorConstants.kMaxVelocityMetersPerSecond,
                    constants.ElevatorConstants.kMaxAccelerationMetersPerSecSquared,
                ),
            ),
            0,
        )

        self.rightelevmotor = phoenix6.hardware.TalonFX(constants.ElevatorConstants.kRightMotorPort)
        self.leftelevmotor = phoenix6.hardware.TalonFX(constants.ElevatorConstants.kLeftMotorPort)
        self.elevencoder = wpilib.Encoder(0, 1,)
        self.feedforward = wpimath.controller.ElevatorFeedforward(
            constants.ElevatorConstants.kSVolts,
            constants.ElevatorConstants.kGVolts,
            constants.ElevatorConstants.kVVoltSecondPerMeter,
            constants.ElevatorConstants.kAVoltSecondSquaredPerMeter,
        )

        self.elevencoder.setDistancePerPulse(
            constants.ElevatorConstants.kEncoderDistancePerPulse
        )

        # Start elevator at rest in neutral position
        self.setGoal(constants.ElevatorConstants.kElevatorOffsetMeters)

    def _useOutput(
        self, output: float, setpoint: wpimath.trajectory.TrapezoidProfile.State
    ) -> None:
        # Calculate the feedforward from the setpoint
        feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)

        # Add the feedforward to the PID output to get the motor output
        self.rightelevmotor.setVoltage(output + feedforward)
        self.leftelevmotor.setVoltage(output + feedforward)
    def _getMeasurement(self) -> float:
        return self.elevencoder.getDistance() + constants.ElevatorConstants.kElevatorOffsetMeters
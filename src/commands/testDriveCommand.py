from commands2 import Command
from wpimath.geometry import Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from phoenix6 import swerve


class TestDriveCommand(Command):
    def __init__(self, drivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self._drive = (
            swerve.requests.FieldCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY
            )
        )
        self.ChassisSpeeds = ChassisSpeeds(1, 1, 0)

    def execute(self):
        print(f"TestDriveCommand execute from {self.drivetrain.get_state().pose}")
        self.drivetrain.apply_request(self._drive.with_velocity_x(1))
        
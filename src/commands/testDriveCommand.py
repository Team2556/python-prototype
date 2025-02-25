from commands2 import Command
from wpimath.geometry import Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from phoenix6 import swerve
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain as csd


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
        self._forward_straight = (swerve.requests.RobotCentric().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY #.OPEN_LOOP_VOLTAGE
            ))

    def execute(self):
        pass


    def thecommand(self):    # print(f"TestDriveCommand execute from {self.drivetrain.get_state().pose}")
        return self._drive.with_velocity_x(0.5)
        # self.drivetrain.apply_request(lambda: swerve.requests.FieldCentric()
        #     .with_drive_request_type(
        #         swerve.SwerveModule.DriveRequestType.VELOCITY
        #     ))
    # def apply(self, something, something_else):
    #     print(f"TestDriveCommand apply get {something} and {something_else} ------------------------------------------------------")
    #     self._drive.with_velocity_x(0.5)
        
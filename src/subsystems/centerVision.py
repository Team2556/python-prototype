# Replace '10.TE.AM.11' with the actual IP address of your Limelight
limelight = Limelight(address='10.TE.AM.11')
import wpilib
import wpimath.controller
from subsystems import command_swerve_drivetrain
import limelight
from wpilib import SmartDashboard
from wpilib.shuffleboard import Shuffleboard
from wpimath.geometry import Pose2d, Transform2d

class LimelightSubsystem:
    def __init__(self, drivetrain):
        """
        Initializes the Limelight subsystem.
        :param drivetrain: The swerve drivetrain subsystem for movement control.
        """
        self.drivetrain = command_swerve_drivetrain
        self.limelight = limelight.Limelight("limelight")
        self.controller = wpimath.controller.PIDController(0.02, 0, 0.001)
        self.target_found = False

    def get_target_offset(self):
        """Retrieves the horizontal offset from the Limelight."""
        result = self.limelight.get_latest_result()
        if result.has_targets():
            self.target_found = True
            return result.get_best_target().get_yaw()
        else:
            self.target_found = False
            return None

    def center_on_apriltag(self):
        """Centers the robot on the detected AprilTag."""
        offset = self.get_target_offset()
        if offset is not None:
            correction = self.controller.calculate(offset, 0)
            self.drivetrain.drive(0, 0, correction)  # Rotate to center on tag
        else:
            self.drivetrain.stop()

    def periodic(self):
        """Called periodically to update the Limelight tracking."""
        self.center_on_apriltag()
        SmartDashboard.putBoolean("Target Found", self.target_found)

    def enable(self):
        """Enable the Limelight LED for tracking."""
        self.limelight.set_led_mode(3)

    def disable(self):
        """Disable the Limelight LED."""
        self.limelight.set_led_mode(1)
    #pull results
    #divide tX by 2, 
    #If greater than half of camera space, turn to left until within 15 pixels
#If less than half, turn to the right until within 15 pixels
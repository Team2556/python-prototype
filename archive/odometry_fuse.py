from commands2 import Command
from phoenix6.utils import fpga_to_current_time
from subsystems import command_swerve_drivetrain, limelightSubsystem 

# a drivetrain command

# self.current_pose_swerve = self.drivetrain.get_state().pose #SwerveDriveState.pose #swerve_drive_state.pose
# trust_vision_data, latest_parsed_result = self.limelight.trust_target(self.current_pose_swerve)
# self.vis_odo_fuse_command =  commands2.ConditionalCommand(VisOdoFuseCommand(self.drivetrain, latest_parsed_result),
#                                                           commands2.PrintCommand("Not Updating Odometer with Vision Data"),
#                                                           lambda: trust_vision_data)
class VisOdoFuseCommand(Command):
    
    def __init__(self, drivetrain, limelight, latest_parsed_result):
        super().__init__()
        # self.latest_parsed_result=latest_parsed_result
        self.drivetrain = command_swerve_drivetrain
        self.limelight = limelight
        self.addRequirements(drivetrain, limelight)# don't really tie up the limelight right? , limelight)

    def execute(self):
        '''add_vision_measurement(vision_robot_pose: Pose2d, timestamp: phoenix6.units.second, vision_measurement_std_devs: tuple[float, float, float] | None = None)'''
        self.current_pose_swerve = self.get_state_copy().pose #SwerveDriveState.pose #swerve_drive_state.pose
        # if trust vision data update the drivetrain odometer
        trust_vision_data, latest_parsed_result = self.limelight.trust_target(self.current_pose_swerve)
        print(f'-\n Running the vis-odo-fuse -- current_pose_swerve: {self.current_pose_swerve} -----\nMust Trust if running\n----\n-----------------------------\n-------\n--\n')
        if trust_vision_data:
            self.drivetrain.add_vision_measurement(latest_parsed_result.robot_pose, fpga_to_current_time(latest_parsed_result.timestamp), latest_parsed_result.vision_measurement_std_devs)

    def isFinished(self):
        return False

    def end(self): 
        pass

    def interrupted(self):
        self.end()
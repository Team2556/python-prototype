from commands2 import Command
# a drivetrain command
class VisOdoFuseCommand(Command):
    
    def __init__(self, drivetrain, latest_parsed_result):
        super().__init__()
        self.latest_parsed_result=latest_parsed_result
        self.drivetrain = drivetrain
        # self.limelight = limelight
        self.addRequirements(drivetrain)# don't really tie up the limelight right? , limelight)
        self.current_pose_swerve = self.drivetrain.get_state_copy().pose #SwerveDriveState.pose #swerve_drive_state.pose

    def execute(self):
        # if trust vision data update the drivetrain odometer
        # trust_vision_data, latest_parsed_result = self.limelight.trust_target(self.current_pose_swerve)
        print(f'-\n Running the vis-odo-fuse -- current_pose_swerve: {self.current_pose_swerve} -----\nMust Trust if running\n----\n-----------------------------\n-------\n--\n')
        # if trust_vision_data:
        self.drivetrain.add_vision_measurement(self.latest_parsed_result.robot_pose, self.latest_parsed_result.timestamp, self.latest_parsed_result.vision_measurement_std_devs)
    ''''add_vision_measurement(vision_robot_pose: Pose2d, timestamp: phoenix6.units.second, vision_measurement_std_devs: tuple[float, float, float] | None = None)'''

    def isFinished(self):
        return False

    def end(self): 
        pass

    def interrupted(self):
        self.end()
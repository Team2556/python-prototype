# will create a commands class that will drive the robot to the desired points
# it will use the robot utility pathGenerator to create a path from current pose to the desired point
# the path will be adjusted to the outside of the hexagons
# the robot will drive to the points in the path using the  PathPlannerPath.waypointsFromPoses, PathConstraints, PathPlannerPath
# https://pathplanner.dev/pplib-create-a-path-on-the-fly.html#qa9tuu_20

# this commands2 class will only use the drivtrain as a requirement

import commands2
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from pathplannerlib.path import PathPlannerPath, PathConstraints,  GoalEndState
from pathplannerlib.auto import RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.commands import FollowPathCommand
from robotUtils import pathGeneratorUtil
import math

class DriveOurOwnWay(FollowPathCommand):
    def __init__(self, finish, drivetrain, pathfindingConstraints_global,
                    hex_centers = [Translation2d(x=4.489323, y=4.025900),
                                    Translation2d(x=13.058902, y=4.025900)], 
                    hex_sizes = [.831723, .831723], 
                    projection_distance = 1.0):
        self.addRequirements(drivetrain)
        self.drivetrain = drivetrain
        self.start = self.drivetrain.get_state().pose
        print(f"DriveOurOwnWay is starting init() and got passed {self.start=}, {finish=}/n")
        self.finish = finish
        self.hex_centers = hex_centers
        self.hex_sizes = hex_sizes
        self.projection_distance = projection_distance
        self.pathfindingConstraints_global = pathfindingConstraints_global
        self.path = pathGeneratorUtil.create_path_with_projections(self.start, self.finish, self.hex_centers, self.hex_sizes, self.projection_distance)
        self.waypoints = PathPlannerPath.waypointsFromPoses(self.path)
        self.constraints = self.pathfindingConstraints_global
        # self.constraints.addMaxVelocityConstraint(1.0)
        # self.constraints.addMaxAccelerationConstraint(1.0)
        # self.constraints.addMaxCentripetalAccelerationConstraint(1.0)

        self.ppPath = PathPlannerPath(self.waypoints, 
                                      self.constraints,
                                      None,
                                      GoalEndState(0.0, Rotation2d.fromDegrees(-90))
                                      )
        self.ppPath.preventFlipping = True
        
        super().__init__(
                        self.ppPath,
                          lambda: self.drivetrain.get_state().pose,
                          lambda: self.drivetrain.get_state().speeds,
                          lambda speeds, feedforwards: self.drivetrain.set_control(
                            self.drivetrain._apply_robot_speeds
                            .with_speeds(speeds)
                            .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                            .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)),
                        PPHolonomicDriveController(
                        # PID constants for translation
                        PIDConstants(10.0, 0.0, 0.0),
                        # PID constants for rotation
                        PIDConstants(.004, 0.0, 0.0)
                        ),
                        RobotConfig.fromGUISettings(),
                        # Assume the path needs to be flipped for Red vs Blue, this is normally the case
                        #  don't know what the 'or' is for ... lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
                        lambda: False,
                        self.drivetrain
                        
                          )
            #path, get_state, get_speeds, set_control, controller, robot_config, is_flipped, drivetrain)

    def execute(self):
        super().execute()

    def isFinished(self):
        return super().isFinished()

    def end(self, interrupted):
        print(f"DrivourOenWay is ending and got passed {interrupted=}")
        super().end(interrupted)

    def interrupted(self):
        super().interrupted()

    def initialize(self):
        # self.drivetrain = drivetrain
        self.start = self.drivetrain.get_state().pose
        # self.finish = self.finish
        print(f"DriveOurOwnWay is starting initialize() and got passed {self.start=}, {self.finish=}/n")
        # self.hex_centers = hex_centers
        # self.hex_sizes = hex_sizes
        # self.projection_distance = projection_distance
        # self.pathfindingConstraints_global = pathfindingConstraints_global
        self.path = pathGeneratorUtil.create_path_with_projections(self.start, self.finish, self.hex_centers, self.hex_sizes, self.projection_distance)
        self.waypoints = PathPlannerPath.waypointsFromPoses(self.path)
        self.constraints = self.pathfindingConstraints_global
        # self.constraints.addMaxVelocityConstraint(1.0)
        # self.constraints.addMaxAccelerationConstraint(1.0)
        # self.constraints.addMaxCentripetalAccelerationConstraint(1.0)

        self.ppPath = PathPlannerPath(self.waypoints, 
                                      self.constraints,
                                      None,
                                      GoalEndState(0.0, Rotation2d.fromDegrees(-90))
                                      )
        self.ppPath.preventFlipping = True
        super().__init__(self.ppPath,
                          lambda: self.drivetrain.get_state().pose,
                          lambda: self.drivetrain.get_state().speeds,
                          lambda speeds, feedforwards: self.drivetrain.set_control(
                            self.drivetrain._apply_robot_speeds
                            .with_speeds(speeds)
                            .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                            .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)),
                            PPHolonomicDriveController(
                            # PID constants for translation
                            PIDConstants(10.0, 0.0, 0.0),
                            # PID constants for rotation
                            PIDConstants(.004, 0.0, 0.0)
                            ),
                            RobotConfig.fromGUISettings(),
                            # Assume the path needs to be flipped for Red vs Blue, this is normally the case
                            #  don't know what the 'or' is for ... lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
                            lambda: False,
                            self.drivetrain)
        super().initialize()

    def schedule(self):
        super().schedule()

    def setNext(self, next_command):
        super().setNext(next_command)

    def setRunWhenDisabled(self, run):
        super().setRunWhenDisabled(run)

    def setStart(self, start_command):
        super().setStart(start_command)

    def setSubsystem(self, subsystem):
        super().setSubsystem(subsystem)



class DriveOurOwnWay2(commands2.Command):
    def __init__(self, start, finish, drivetrain, autobuilder ,pathfindingConstraints_global,
                    hex_centers = [Translation2d(x=4.489323, y=4.025900),
                                    Translation2d(x=13.058902, y=4.025900)], 
                    hex_sizes = [0.831723, 0.831723], 
                    projection_distance = .9,
                    ):
        '''Projection_distance is beyond the edge of the reef'''
        super().__init__()
        self.drivetrain = drivetrain
        self.autobuilder = autobuilder
        self.addRequirements(self.drivetrain)
        self.start = start
        self.finish = finish
        self.hex_centers = hex_centers
        self.hex_sizes = hex_sizes
        self.projection_distance = projection_distance
        self.pathfindingConstraints_global = pathfindingConstraints_global

    def initialize(self):
        self.start = self.drivetrain.get_state().pose
        self.path = pathGeneratorUtil.create_path_with_projections(self.start, self.finish, self.hex_centers, self.hex_sizes, self.projection_distance)
        self.waypoints = PathPlannerPath.waypointsFromPoses(self.path)
        self.constraints = self.pathfindingConstraints_global
        # self.constraints.addMaxVelocityConstraint(1.0)
        # self.constraints.addMaxAccelerationConstraint(1.0)
        # self.constraints.addMaxCentripetalAccelerationConstraint(1.0)

        self.ppPath = PathPlannerPath(self.waypoints, 
                                      self.constraints,
                                      None,
                                      GoalEndState(0.0, Rotation2d.fromDegrees(-90))
                                      )
        self.ppPath.preventFlipping = True
        # self.ppPath.calculate()
        # self.ppPath.smooth()
        # self.ppPath.optimize()
        # self.ppPath.sample()
        # self.ppPath.reset()
        self.followCommand = FollowPathCommand( self.ppPath,
                          lambda: self.get_state().pose,
                          lambda: self.get_state().speeds,
                          lambda speeds, feedforwards: self.set_control(
                            self._apply_robot_speeds
                            .with_speeds(speeds)
                            .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                            .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)),
                        PPHolonomicDriveController(
                        # PID constants for translation
                        PIDConstants(10.0, 0.0, 0.0),
                        # PID constants for rotation
                        PIDConstants(.004, 0.0, 0.0)
                        ),
                        RobotConfig.fromGUISettings(),
                        # Assume the path needs to be flipped for Red vs Blue, this is normally the case
                        #  don't know what the 'or' is for ... lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
                        lambda: False,
                        self.drivetrain
                        
                          )


    def execute(self):
        
        FollowPathCommand( self.ppPath,
                          lambda: self.get_state().pose,
                          lambda: self.get_state().speeds,
                          lambda speeds, feedforwards: self.set_control(
                            self._apply_robot_speeds
                            .with_speeds(speeds)
                            .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                            .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)),
                        PPHolonomicDriveController(
                        # PID constants for translation
                        PIDConstants(10.0, 0.0, 0.0),
                        # PID constants for rotation
                        PIDConstants(.004, 0.0, 0.0)
                        ),
                        RobotConfig.fromGUISettings(),
                        # Assume the path needs to be flipped for Red vs Blue, this is normally the case
                        #  don't know what the 'or' is for ... lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
                        lambda: False,
                        self.drivetrain
                        
                          )
        # self.drivetrain.followPath(self.ppPath)

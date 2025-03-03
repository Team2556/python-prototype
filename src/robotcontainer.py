#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2

import commands2.button, commands2.cmd
import numpy as np
from commands2.sysid import SysIdRoutine
import wpimath.trajectory

from constants import ClimbConstants
from generated.tuner_constants import TunerConstants
from constants import RobotDimensions, ElevatorConstants
from subsystems import (
    # ElevatorSubsystem,
    #coralSubsystem,
    limelight,
    # pneumaticSubsystem,
    # oneMotor,
    ultrasonics, #ClimbSubsystem
)
from telemetry import Telemetry
from robotUtils import controlAugment, pathGeneratorUtil

from pathplannerlib.auto import AutoBuilder, PathfindThenFollowPath, PathPlannerAuto
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState , IdealStartingState
from phoenix6 import swerve
from phoenix6.hardware import TalonFX
import wpilib
import wpimath
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Rotation2d, Translation2d, Transform2d, Pose2d, Rectangle2d
from wpimath.units import (
    rotationsToRadians,
    degrees,
    radians,
    degreesToRadians,
    radiansToDegrees,
    metersToInches,
    inchesToMeters,
)
import wpinet
import math
from commands.odometrySnap2Line import SnapToLineCommand
from commands.driveOurOwnWay import DriveOurOwnWay
from commands.driveWithForceCommand import DriveToPointForce
from commands.testDriveCommand import TestDriveCommand

# from commands.gotoClosestPath import GotoClosestPath
# from commands.drive_one_motor import DriveOneMotorCommand
from commands.liftElevator import LiftElevatorCommand
# from commands import coralCommand
import networktables as nt
from networktables import util as ntutil

from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig
from wpimath.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians


# from subsystems import algae


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.robotWidthBumpered = inchesToMeters(RobotDimensions.WIDTH_w_bumpers)
        SmartDashboard.putNumber("Max Speed", TunerConstants.speed_at_12_volts)
        SmartDashboard.putNumber("Elevator/Kp", ElevatorConstants.kElevatorKp)
        SmartDashboard.putNumber("Elevator/Ki", ElevatorConstants.kElevatorKi)
        SmartDashboard.putNumber("Elevator/Kd", ElevatorConstants.kElevatorKd)
        SmartDashboard.putNumber("Elevator/Kg", ElevatorConstants.kGVolts)
        # SmartDashboard.putNumber("Elevator/Kf",0.0)

        self.timer = wpilib.Timer()
        self.timer.start()

        self._max_speed = SmartDashboard.getNumber(
            "Max Speed", TunerConstants.speed_at_12_volts
        )
        """speed_at_12_volts desired top speed"""
        print(f"Max speed: {self._max_speed}")
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.05)
            .with_rotational_deadband(
                self._max_angular_rate * 0.05
            )  # Add a 5% deadband on output
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY #OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive
        )
        """
        Control the drive motor using a velocity closed-loop request.
        The control output type is determined by SwerveModuleConstants.DriveMotorClosedLoopOutput
        """


        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY #.OPEN_LOOP_VOLTAGE
            )
        )

        self.drivetrain = TunerConstants.create_drivetrain()

        self.hex_centers = [Translation2d(x=4.489323, y=4.025900),
                                    Translation2d(x=13.058902, y=4.025900)], 
        self.hex_sizes = [.831723, .831723]
        self.config_traj = TrajectoryConfig(self._max_speed, 2.0)
        self.config_traj.setKinematics(self.drivetrain.kinematics)
        self.holonomic_controller = HolonomicDriveController(
            PIDController(1.0, 0, 0),
            PIDController(1.0, 0, 0),
            ProfiledPIDControllerRadians(1.0, 0, 0, wpimath.trajectory.TrapezoidProfileRadians.Constraints(self._max_speed,self._max_angular_rate))
        )
        # self._driveToPointForce = DriveToPointForce(drivetrain=self.drivetrain, 
        #                                                          target=Pose2d(2, 2, Rotation2d(0)), 
        #                                                          obstacle=self.hex_centers[0][0],
        #                                                          max_speed=self._max_speed,
        #                                                          max_angular_rate=self._max_angular_rate,
        #                                                          )

        # self.algae = algae.AlgaeHandler()
        # self.ultrasonic = ultrasonics.Ultrasonics()

        self._logger = Telemetry(self._max_speed)

        # This one's probably used for moving
        self._joystick = commands2.button.CommandXboxController(0)
        # This one's probably used for scoring stuff
        self._joystick2 = commands2.button.CommandXboxController(1)

        # Using NetworkButton with a USB keyboard will require running a seperate python program on the driver's station
        # python ../DriverstationUtils/keyboard_to_nt.py
        # pressing {CTRL+C} will stop the program

        self._keyboard_dock_left_feeder = commands2.button.NetworkButton(
            "/SmartDashboard/keyboard", "l"
        )
        self._keyboard_reset_odometry_by_drivers = commands2.button.NetworkButton(
            "/SmartDashboard/keyboard", "a"
        )  # SmartDashboard.getBoolean("/SmartDashboard/keyboard/a", False)
        self.keyboard_goto_position_7_7 = commands2.button.NetworkButton(
            "/SmartDashboard/keyboard", "7"
        )


        # self.coral_track = coralSubsystem.CoralTrack()
        pneumaticENABLE = False
        if pneumaticENABLE:
            self.pneumaticsHub = pneumaticSubsystem.PneumaticSubsystem()

        # self.climb = ClimbSubsystem.ClimbSubsystem()
        # self.one_motor = oneMotor.OneMotor(
        #     motor=[TalonFX(constants.CAN_Address.FOURTEEN),TalonFX(constants.CAN_Address.FIFTEEN)]   )
        # section elevator
        self.ENABLE_ELEVATOR = False
        if self.ENABLE_ELEVATOR:
            self.elevator = ElevatorSubsystem.ElevatorSubsystem()
            self._reset_zero_point_here = self.elevator.reset_zero_point_here()
            self._elevator_motors_break = self.elevator.elevator_motors_break
        # endsection elevator

        # Vision
        self.limelight = limelight.LimelightSubsystem()
        for port in np.arange(start=5800, stop=5809):
            wpinet.PortForwarder.getInstance().add(port, "limelight.local", port)

        # self.coral_command = coralCommand.CoralCommand(
        #     self.coral_track, self.pneumaticsHub, self.elevator, self.timer
        # )

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("SetOdo_DriverWallRtFeeder")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)
        self.test_drive = TestDriveCommand(self.drivetrain)

        self.path_doc_proc_short = PathPlannerPath.fromPathFile("Dock-Processor")
        self.path_doc_proc_midfield = PathPlannerPath.fromPathFile("Dock-Proc-Mid")
        self.path_doc_proc_RtWall = PathPlannerPath.fromPathFile("Dock-Proc-RtWall")
        self.pathlist_dock_processing = [
            self.path_doc_proc_short,
            self.path_doc_proc_midfield,
            self.path_doc_proc_RtWall,
        ]
        self.path_doc_feed_right = PathPlannerPath.fromPathFile("Dock-Feed-Right")
        self.path_doc_feed_left = PathPlannerPath.fromPathFile("Dock-Feed-Left")
        self.pathlist_dock_feed = [self.path_doc_feed_right, self.path_doc_feed_left]

        self.pathfinding_constraints_global = PathConstraints(
            3 / 5,
            4 / 4,
            degreesToRadians(540 / 10),
            degreesToRadians(720 / 10),
            12,
            False,
        )  # was:(3.0, 4.0, degreesToRadians(540), degreesToRadians(720),12,False)

        # section TeleAuto coral
        # TODO: implement or replace with a better method
        # define numpy array of coral locations; there are 12 poles with 4 levels available for scoring for a total fof 48 scoring locations
        self.coral_locations = np.zeros((12, 4))

        # np.array([(x, y) for x, y in zip(range(12), range(4) * 12)])
        def record_score_coral(pole: int, level: int):
            assert 0 <= pole < 12
            assert 0 <= level < 4
            self.coral_locations[pole, level] = 1

        def record_descore_coral(pole: int, level: int):
            assert 0 <= pole < 12
            assert 0 <= level < 4
            self.coral_locations[pole, level] = 0

        def pick_coral_to_score():
            # pick the first coral location that is not scored
            for level in range(3, 0, -1):
                for pole in range(12):
                    if self.coral_locations[pole, level] == 0:
                        return pole, level

            return None

        # endsection TeleAuto coral

        # self.path_dock_processing_command = AutoBuilder.pathfindThenFollowPath(
        #     goal_path= self.closest_path_to_robot,
        #     pathfinding_constraints=PathConstraints(3.0, 4.0, degreesToRadians(540), degreesToRadians(720),12,False),
        #     # rotation_delay_distance=0.5, online example bad
        # )

        # PathfindThenFollowPath()
        self.closest_proc_path_to_robot_lam = lambda pose: min(
            self.pathlist_dock_processing,
            key=lambda path: path._waypoints[0].anchor.distance(pose),
        )
        # self.closest_proc_path_to_robot = self.closest_proc_path_to_robot_lam(self.drivetrain.get_state().pose.translation())
        self.closest_feed_path_to_robot_lam = lambda pose: min(
            self.pathlist_dock_feed,
            key=lambda path: path._waypoints[0].anchor.distance(pose),
        )
        # self.closest_feed_path_to_robot = self.closest_feed_path_to_robot_lam(self.drivetrain.get_state().pose.translation())

        self.set_closest_paths(self.drivetrain.get_state().pose)

        # Configure the button bindings
        self.configureButtonBindings()

    def set_closest_paths(self, pose: Pose2d) -> None:
        self.closest_proc_path_to_robot = self.closest_proc_path_to_robot_lam(
            pose.translation()
        )
        self.closest_feed_path_to_robot = self.closest_feed_path_to_robot_lam(
            pose.translation()
        )
    def calculate_angle(self, p1, p2):
        """Calculate the angle between two points."""
        return np.arctan2(p2[1] - p1[1], p2[0] - p1[0])

    def generate_pose2d_path(self, points):
        """Convert a list of (x, y) points into a list of Pose2d with averaged rotation angles."""
        poses = []
        num_points = len(points)
        
        # this should be implementing a 'turn short' path by averaging the angles of the previous and next path legs
        # but it may lead to a 'fly over' where the robot urns out a little before the turn and goes a little wide after
        for i in range(num_points):
            if i == 0:
                angle = self.calculate_angle(points[i], points[i + 1])
            elif i == num_points - 1:
                angle = self.calculate_angle(points[i - 1], points[i])
            else:
                angle1 = self.calculate_angle(points[i - 1], points[i])
                angle2 = self.calculate_angle(points[i], points[i + 1])
                angle = (angle1 + angle2) / 2
            
            poses.append(Pose2d(points[i][0], points[i][1], Rotation2d(angle)))
        
        return poses

    # def build_path(self,current_pose, target: Translation2d, obstacle_center: Translation2d, obstacle_radius: float) -> list[Translation2d]:
    #     # current_pose = AutoBuilder.getCurrentPose() # self.drivetrain.get_state().pose
    #     # path_points = [current_pose.translation()]
        
    #     #generate points for the path, function or hardcoded
    #     path_points = pathGeneratorUtil.subdivide_line(current_pose,target,.5)
    #     path_points = pathGeneratorUtil.adjust_points_to_outside_circle(path_points, self.hex_centers[0][0], self.hex_sizes[0]+1.0)
    
    #     path_poses = self.generate_pose2d_path(path_points)
    #     pp_waypoints = PathPlannerPath.waypointsFromPoses( path_poses)
    #     #'constraints', 'ideal_starting_state', and 'goal_end_state'
    #     pp_path = PathPlannerPath(pp_waypoints, 
    #         constraints=self.pathfinding_constraints_global,
    #         ideal_starting_state=IdealStartingState(.5, current_pose.rotation()),
    #         goal_end_state= GoalEndState(0.0,Rotation2d(0)),
    #         is_reversed= DriverStation.getAlliance() == DriverStation.Alliance.kRed,)

    #     return pp_path



    def create_path_following_command(self, target: Pose2d) -> commands2.Command:
        
        print(f"Creating path to {target} ================================================____--------")
        current_pose = self.drivetrain.get_state().pose
        path = self.drivetrain.build_path( target, self.hex_centers[0][0], self.hex_sizes[0])
        print(f"Path: {path}")
        if not path:
            wpilib.SmartDashboard.putString("Path Status", "No valid path")
            return commands2.InstantCommand()
        
        trajectory = TrajectoryGenerator.generateTrajectory(
            current_pose,
            [p for p in path[1:-1]],
            target,#Pose2d(target, current_pose.rotation()),
            self.config_traj
        )
        command = commands2.SwerveControllerCommand(
            trajectory=trajectory,
            pose=self.drivetrain.get_state(),
            kinematics=self.drivetrain.kinematics,
            controller=self.holonomic_controller,
            outputModuleStates= lambda: self.drivetrain.get_state().speeds,
            # self.drivetrain.set_module_states,
            # lambda speeds: #print(speeds
            # self.drivetrain.set_control(
            #     self.drivetrain._apply_robot_speeds
            #     .with_speeds(ChassisSpeeds(speeds,)
            # ),

            # outputModuleStates=lambda speeds: self.drivetrain.kinematics.toChassisSpeeds(speeds),  # Use kinematics to convert module states to chassis speeds
            requirements=[self.drivetrain],
            # desiredRotation=lambda: target.rotation(),
        

        )
        wpilib.SmartDashboard.putString("Path Status", f"Following path to {target}")
        return command


    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        # Drivetrain will execute this command periodically

        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -controlAugment.smooth(
                            self._joystick.getLeftY(), exponential_for_curving=3
                        )
                        * self._max_speed
                        * self.invertBlueRedDrive
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -controlAugment.smooth(
                            self._joystick.getLeftX(), exponential_for_curving=3
                        )
                        * self._max_speed
                        * self.invertBlueRedDrive
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -controlAugment.smooth(self._joystick.getRightX())
                        * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                    .with_center_of_rotation(
                        Translation2d(
                            x=self.robotWidthBumpered
                            * 0.2
                            * (
                                controlAugment.smooth(
                                    controlAugment.one_side_control_only(
                                        self._joystick.getRightY(), "Pos"
                                    )
                                )
                            ),
                            # want y translation to depend on direction of turn
                            y=math.copysign(1, self._joystick.getRightX()),
                        )
                        * self.robotWidthBumpered
                        * 0.2
                        * (
                            controlAugment.smooth(
                                controlAugment.one_side_control_only(
                                    self._joystick.getRightY(), "Pos"
                                )
                            )
                        )
                    )
                    # shift the center of rotation to opposite front corner, if the driver pulls down on the right stick in addition to the side.
                    # This should allow some nice defensive roll-off maneuvers
                )
            )
        )

        # self.one_motor.setDefaultCommand(DriveOneMotorCommand(self.one_motor, self._joystick2))
        if self.ENABLE_ELEVATOR:
            self.elevator.setDefaultCommand(
                LiftElevatorCommand(self.elevator, self._joystick2)
            )
            (self._joystick2.start() & self._joystick2.a()).whileTrue(
                self._reset_zero_point_here
            )  # .onFalse(lambda: self._elevator_motors_break) #TODO: fix this to not crash :)
            # (self._joystick2.start() & self._joystick2.x()).whileTrue(lambda: self._reset_zero_point_here) #TODO: fix this to not crash :)

        # section vision related commands

        self._joystick.x().onTrue(SnapToLineCommand(self.drivetrain))

        # endsection vision related commands

        # Coral Track controls
        # self.coral_track.setDefaultCommand(self.coral_command)

        # self._joystick2.x().whileTrue(
        #     commands2.cmd.run(
        #         lambda: self.coral_command.enable_discharge(), self.coral_track
        #     )
        # )

        # self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(
                        -self._joystick.getLeftY() * self.invertBlueRedDrive,
                        -self._joystick.getLeftX() * self.invertBlueRedDrive,
                    )
                )
            )
        )
        # Climb Controls
        # self._joystick2.povUp().whileTrue(
        #     self.climber.set_desired_state_command(ClimbSubsystem.CLIMB_POSITIVE)
        # ).onFalse(
        #     self.climber.set_desired_state_command(ClimbSubsystem.STOP)
        # )

        # self._joystick2.povDown().whileTrue(
        #     self.climber.set_desired_state_command(ClimbSubsystem.CLIMB_NEGATIVE)
        # ).onFalse(
        #     self.climber.set_desired_state_command(ClimbSubsystem.STOP)
        # )
        # Algae controls with controller 2 left joystick
        # It's negative because that's how xbox controllers work
        """self.algae.setDefaultCommand(
            commands2.cmd.run(
                lambda: self.algae.cycle(self._joystick2.getRightY() * -1), self.algae
            )
        )"""

        # self._joystick.pov(0).whileTrue(
        #     self.drivetrain.apply_request(
        #         lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0)
        #     )
        # )
        # trim out the gyro drift; if press POV 0 and move right stick update the drivetrain rotation, but conditional that the right stick input is more than .1
        self._joystick.pov(0).whileTrue(
            commands2.ConditionalCommand(
                self.drivetrain.apply_request(
                    lambda: self.drivetrain.reset_rotation(
                        self.drivetrain.get_rotation3d().toRotation2d()
                        + Rotation2d(
                            0, 0, degreesToRadians(-self._joystick.getRightX())
                        )
                    )
                ),
                self.drivetrain.apply_request(
                    lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(
                        0
                    )
                ),
                lambda: self._joystick.getRightX().__abs__() > 0.1,
            )
        )
        self._joystick.pov(180).whileTrue(
            commands2.ConditionalCommand(
                self.drivetrain.apply_request(
                    lambda: self.drivetrain.reset_rotation(
                        self.drivetrain.get_rotation3d().toRotation2d()
                        + Rotation2d(0, 0, degreesToRadians(self._joystick.getRightX()))
                    )
                ),
                self.drivetrain.apply_request(
                    lambda: self._forward_straight.with_velocity_x(
                        -0.5
                    ).with_velocity_y(0)
                ),
                lambda: self._joystick.getRightX().__abs__() > 0.1,
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._joystick.back() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.back() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_rotation(
                    Rotation2d(
                        np.pi
                        * (DriverStation.getAlliance() == DriverStation.Alliance.kRed)
                    )
                )
            )
        )
        # self.drivetrain.seed_field_centric()))
        (self._joystick.leftBumper() & self._joystick.a()).onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose_by_zone(zone="a")
            )
        )
        (self._joystick.leftBumper() & self._joystick.b()).onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose_by_zone(zone="b")
            )
        )
        (self._joystick.leftBumper() & self._joystick.x()).onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose_by_zone(zone="x")
            )
        )
        (self._joystick.leftBumper() & self._joystick.y()).onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose_by_zone(zone="y")
            )
        )

        # section Autonomous During Teleop

        # This method used rectangles as triggers
        # teleAuto to processing
        # TODO: handel the red side of the field; more rectangles or a function that returns the rotation of the rectangle
        # self.rotate_rectangle_by = Rotation2d( degreesToRadians(0))
        # if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        #     self.rotate_rectangle_by = Translation2d(Rotation2d(degreesToRadians(180)))
        rect_feedArea = Rectangle2d(
            Translation2d(0, 0), Translation2d(4.50, 7.50)
        )  # .transformBy(self.rotate_rectangle_by)
        rect_procArea = Rectangle2d(
            Translation2d(0, 0), Translation2d(8.70, 1.10)
        )  # .transformBy(self.rotate_rectangle_by)
        # print(f'{rect_feedArea.contains(AutoBuilder.getCurrentPose().translation())=}')

        
        (
            self._joystick.b()
            & self._joystick.povDown()
            & commands2.button.Trigger(
                lambda: rect_feedArea.contains(
                    AutoBuilder.getCurrentPose().translation()
                )
                or rect_procArea.contains(AutoBuilder.getCurrentPose().translation())
            )
        ).onTrue(
            AutoBuilder.pathfindThenFollowPath(
                goal_path=self.path_doc_proc_RtWall,
                pathfinding_constraints=self.pathfinding_constraints_global,
            )
        )
        rect_midArea = Rectangle2d(
            Translation2d(6.5, 2.7), Translation2d(8.7, 7.70)
        )  # .transformBy(self.rotate_rectangle_by)
        rect_topFarArea = Rectangle2d(
            Translation2d(4.5, 5.5), Translation2d(8.7, 7.70)
        )  # .transformBy(self.rotate_rectangle_by)
        (
            self._joystick.b()
            & self._joystick.povDown()
            & commands2.button.Trigger(
                lambda: rect_midArea.contains(
                    AutoBuilder.getCurrentPose().translation()
                )
                or rect_topFarArea.contains(AutoBuilder.getCurrentPose().translation())
            )
        ).onTrue(
            AutoBuilder.pathfindThenFollowPath(
                goal_path=self.path_doc_proc_midfield,
                pathfinding_constraints=self.pathfinding_constraints_global,
            )
        )
        # teleAuto to feeders
        rect_rightFeedArea = Rectangle2d(
            Translation2d(0, 0), Translation2d(8.70, 4.0)
        )  # .transformBy(self.rotate_rectangle_by)
        (
            self._joystick.a()
            & self._joystick.povDown()
            & commands2.button.Trigger(
                lambda: rect_rightFeedArea.contains(
                    AutoBuilder.getCurrentPose().translation()
                )
            )
        ).onTrue(
            AutoBuilder.pathfindThenFollowPath(
                goal_path=self.path_doc_feed_right,
                pathfinding_constraints=self.pathfinding_constraints_global,
            )
        )

        rect_leftFeedArea = Rectangle2d(
            Translation2d(0, 4.5), Translation2d(8.70, 7.70)
        )  # .transformBy(self.rotate_rectangle_by)
        # can make the Networktables button an alternative to joystick button
        (
            (
                self._keyboard_dock_left_feeder.or_(
                    self._joystick.a() & self._joystick.povDown()
                )
            )
            & commands2.button.Trigger(
                lambda: rect_leftFeedArea.contains(
                    AutoBuilder.getCurrentPose().translation()
                )
            )
        ).onTrue(
            AutoBuilder.pathfindThenFollowPath(
                goal_path=self.path_doc_feed_left,
                pathfinding_constraints=self.pathfinding_constraints_global,
            )
        )
        # Can make the Networktables button trigger an action
        self._keyboard_reset_odometry_by_drivers.onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose_by_zone(zone="a")
            )
        )
        
        self.keyboard_goto_position_7_7.onTrue(  
            # TODO: see about following our own generated path
            AutoBuilder.followPath(pathGeneratorUtil.build_path(current_pose= AutoBuilder.getCurrentPose(),
                                   target= Translation2d(6, 7),
                                    obstacle_center= self.hex_centers[0][0],
                                     avoidance_radius= self.hex_sizes[0]+1.0,
                                     pathfinding_constraints= self.pathfinding_constraints_global)
            )

            #self.create_path_following_command(Pose2d(Translation2d(7, 7),Rotation2d(0)))
            # self.drivetrain.apply_request(
            #     lambda: 
            #         DriveToPointForce(drivetrain=self.drivetrain, 
            #                                 target=Pose2d(2, 2, Rotation2d(0)), 
            #                                 obstacle=self.hex_centers[0][0],
            #                                 obstacle_radius=2.0,
            #                                 max_speed=self._max_speed,
            #                                 max_angular_rate=self._max_angular_rate,
            #                                 k_attr=1.0,
            #                                 k_rep=20.0,
            #                                 influence_radius=5.25,
            #                                 tolerance=.05
            #                                 ).execute_it()
            # )
            )

                
        """#this method uses the robot periodic updated closest path to robot
        pathfinding_constraints_global = PathConstraints(3/3, 4/3, degreesToRadians(540/2), degreesToRadians(720/2),12,False)#was:(3.0, 4.0, degreesToRadians(540), degreesToRadians(720),12,False)
        # teleAuto to processing
        (self._joystick.b() & self._joystick.povDown()).whileTrue(AutoBuilder.pathfindThenFollowPath( goal_path= self.closest_proc_path_to_robot,
                                                                                                     pathfinding_constraints=pathfinding_constraints_global))
        # teleAuto to feeders
        (self._joystick.a() & self._joystick.povDown()).whileTrue(AutoBuilder.pathfindThenFollowPath( goal_path= self.closest_feed_path_to_robot,
                                                                                                     pathfinding_constraints=pathfinding_constraints_global))
        """
        # configured in Command_swerve_drivetrain AutoBuilder.configure
        # (self._joystick.a() & self._joystick.povDown()).whileTrue( GotoClosestPath(drivetrain=self.drivetrain,
        #                                                                            paths=self.pathlist_dock_processing))

        # endsection Autonomous During Teleop

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # return commands2.cmd.print_("No autonomous command configured")
        return self._auto_chooser.getSelected()
 
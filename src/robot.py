#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import wpilib
import commands2
import typing
from wpimath.geometry import Pose2d, Transform2d, Rotation2d, Translation2d
from wpilib import SmartDashboard, DriverStation, Field2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.logging import PathPlannerLogging
from phoenix6.utils import fpga_to_current_time


from robotcontainer import RobotContainer

class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """
       

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()

        #region Glass field viewer
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)
        #endregion Glass field viewer
        # Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback(lambda pose: self.field.setRobotPose(pose))

        # Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback(lambda pose: self.field.getObject('target pose').setPose(pose))

        # Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback(lambda poses: self.field.getObject('path').setPoses(poses))
        

 
    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""


        # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        # commands, running already-scheduled commands, removing finished or interrupted commands,
        # and running subsystem periodic() methods.  This must be called from the robot's periodic
        # block in order for anything in the Command-based framework to work.
        commands2.CommandScheduler.getInstance().run()
        
        self.container._max_speed = SmartDashboard.getNumber("Max Speed",0.0)
        
        self.container.invertBlueRedDrive = 1
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.container.invertBlueRedDrive = -1

        #section vision related commands
        #update swerve odometry with vision data
        #attmept to add in vision update

        '''add_vision_measurement(vision_robot_pose: Pose2d, timestamp: phoenix6.units.second, vision_measurement_std_devs: tuple[float, float, float] | None = None)'''
        # WHy won't this work in command_swerve_drivetrain ?? -- need to get the container instantiated limelight, but that would create a circular reference
        
        current_pose_swerve = self.container.drivetrain.get_state_copy().pose #SwerveDriveState.pose #swerve_drive_state.pose
        # if trust vision data update the drivetrain odometer
        
        
        trust_vision_data, viz_pose, latest_parsed_result, time_of_measurement = self.container.limelight.trust_target(current_pose_swerve)
        if trust_vision_data:
            if latest_parsed_result:
                # vision_measurement_std_devs = (.1, .1, 45*6/180) # {latest_parsed_result.vision_measurement_std_devs} #TODO: look into using MegaTag to get stddevs
                print(f'-\n Running the robot periodic -- current_pose_delta: {current_pose_swerve - viz_pose} -----\n The vision stuff{latest_parsed_result.botpose_wpiblue=} With uncertainty ...need to use megatag\n----\n------Pose update {current_pose_swerve - self.container.drivetrain.get_state_copy().pose=}-----------------------\n-------\n--\n')
                # self.container.drivetrain.set_vision_measurement_std_devs((.01, .01, 15*6/180))
                # self.container.drivetrain.add_vision_measurement(viz_pose, fpga_to_current_time(time_of_measurement)) #, vision_measurement_std_devs)
                # After_viz_update_odo_pose = self.container.drivetrain.get_state().pose

                # self.container.drivetrain.odometry_thread.set_thread_priority(99)
                # self.container.drivetrain.OdometryThread.stop(self.container.drivetrain.odometry_thread)
                # self.container.drivetrain.reset_pose(After_viz_update_odo_pose)

                self.container.drivetrain.use_vision(viz_pose, fpga_time_of_measurement=fpga_to_current_time(time_of_measurement))#(After_viz_update_odo_pose)

                # self.container.drivetrain.OdometryThread.start(self.container.drivetrain.odometry_thread)
                # self.container.drivetrain.odometry_thread.set_thread_priority(20)

                # print(f"Why won't it stick: {After_viz_update_odo_pose-viz_pose} Vs what was {current_pose_swerve - viz_pose} ????????????????????????????????????????????????????????????")
                
        else:
            pass#print(f'-\n Running the robot periodic NOT TRUSTING VISION -- -----\n----\n------------XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX-----------------\n-------\n--\n')
     
        #endsection vision related commands
        

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed: 
            AutoBuilder._shouldFlipPath = lambda: True
        else:
            AutoBuilder._shouldFlipPath = lambda: False
        pass

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()


        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        # Run the VisOdoFuseCommand ???
        pass

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        #section update closest paths
        current_pose_swerve = self.container.drivetrain.get_state_copy().pose
        #self.container.set_closest_paths(current_pose_swerve)
        #print(f'closest_proc_path_to_robot: {self.container.closest_proc_path_to_robot}')
                
        # self.container.elevator.elevatorPeriodic()

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
import typing
from wpilib import SmartDashboard, DriverStation, Field2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.logging import PathPlannerLogging


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
                # The origin is always blue. When our alliance is red, X and Y need to be inverted
        self.container.invertBlueRedDrive = 1
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.container.invertBlueRedDrive = -1
        # print(f"Robot is on the {DriverStation.getAlliance()} alliance.\n -\n-\n-\n- {self.container.invertBlueRedDrive =}")

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
        
        pass

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

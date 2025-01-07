#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import wpilib.drive

import commands2
from commands2 import CommandScheduler

from commands.drivetrain import DriveTrain
from constants import *


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        CommandScheduler.getInstance().run()
        self.timer = wpilib.Timer()

        # Subsystems
        self.robotDrive = DriveTrain()

        # User Inputs
        self.driveController = commands2.button.CommandXboxController(
            OIConstants.driver_1_port
        )

        self.ConfigureButtonBindings()
        self.ConfigureDefaultCommands()


    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.restart()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous."""
        self.robotDrive.updateOdometry()

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""

    def teleopPeriodic(self) -> None:
        """This function is called periodically during teleoperated mode."""

    def ConfigureButtonBindings(self):
        pass

    def ConfigureDefaultCommands(self):
        self.robotDrive.setDefaultCommand(
            commands2.cmd.run(
                lambda: self.robotDrive.driveWithJoystick(self.driveController)
            ),
            # self.robotDrive,
        )

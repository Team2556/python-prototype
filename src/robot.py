#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import phoenix6
import wpilib
import commands2
import typing
import math
from wpilib import (SmartDashboard, Field2d)
import wpimath.controller
import wpimath.trajectory

from robotcontainer import RobotContainer
from constants import ElevatorConstants




class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """
    class Elevator(wpilib.Elevator):
        ElevatorConstants.kElevatorDrumRadius = 0.0508
        
        # distance per pulse = (distance per revolution) / (pulses per revolution)
        #  = (Pi * D) / ppr
        kElevatorEncoderDistPerPulse = 2.0 * math.pi * 0.0508 / 4096.0

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        self.elevconstraints = wpimath.trajectory.TrapezoidProfile.Constraints(1.75, 0.75)
        self.elevcontroller = wpimath.controller.PIDController(5.0, 0, 0)
        self.elevencoder = wpilib.Encoder(0, 1)
        self.leftelevmotor = phoenix6.hardware.TalonFX(0)
        self.rightelevmotor = phoenix6.hardware.TalonFX(1)
        self.joystick = wpilib.XboxController(0)

        self.elevencoder.setDistancePerPulse(2.0 * math.pi * 0.0508 / 4096.0)

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

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

        if self.joystick.rightTrigger():
            # Here, we run PID control like normal, with a constant setpoint of 30in (0.762 meters).
            pidOutput = self.elevcontroller.calculate(self.elevencoder.getDistance(), 0.762)
            self.leftelevmotor.setVoltage(pidOutput)
            self.rightelevmotor.setVoltage(pidOutput)
        else:
            # Otherwise we disable the motor
            self.leftelevmotor.set(0.0)
            self.rightelevmotor.set(0.0)

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        self.leftelevmotor.set(0)
        self.rightelevmotor.set(0)

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
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

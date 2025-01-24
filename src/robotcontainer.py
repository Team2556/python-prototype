#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button, commands2.cmd
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry
from generated import drive_smoothing

from pathplannerlib.auto import AutoBuilder
from phoenix6 import swerve
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from subsystems import limelight
from commands.odometry_fuse import VisOdoFuseCommand



class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        SmartDashboard.putNumber("Max Speed", TunerConstants.speed_at_12_volts)
        self._max_speed = SmartDashboard.getNumber("Max Speed", TunerConstants.speed_at_12_volts)
        '''self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed'''
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
            )  # Add a 5% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive 

        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )


        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain()

        # Vision
        self.limelight = limelight.LimelightSubsystem()

        # Run the VisOdoFuseCommand --- put it where?
        self.current_pose_swerve = self.drivetrain.get_state_copy().pose #SwerveDriveState.pose #swerve_drive_state.pose
        trust_vision_data, latest_parsed_result = self.limelight.trust_target(self.current_pose_swerve)
        self.vis_odo_fuse_command =  commands2.ConditionalCommand(VisOdoFuseCommand(self.drivetrain, latest_parsed_result),
                                                                  commands2.PrintCommand("Not Updating Odometer with Vision Data"),
                                                                  lambda: trust_vision_data)
        # self.vis_odo_fuse_command.schedule()

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)
        
        # Configure the button bindings
        self.configureButtonBindings()

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
                        -drive_smoothing.smooth(self._joystick.getLeftY()) * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -drive_smoothing.smooth(self._joystick.getLeftX()) * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -drive_smoothing.smooth(self._joystick.getRightX()) * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )#.alongWith(commands2.PrintCommand("Running default command. \nq\nqqq\nqqqqqqq\nqqqqqqqqqqqqqqqqqqqqqqqqqqqqq\nqqqqqqq\n---\n")),
        )
        
        #
        self._joystick.x().negate().whileTrue( self.vis_odo_fuse_command) #.negate()

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )
        
        self._joystick.pov(0).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0)
            )
        )
        self._joystick.pov(180).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(-0.5).with_velocity_y(0)
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
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # return commands2.cmd.print_("No autonomous command configured")
        return self._auto_chooser.getSelected()

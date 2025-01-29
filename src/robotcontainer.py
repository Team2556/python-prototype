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
from robotUtils import controlAugment

from pathplannerlib.auto import AutoBuilder, PathConstraints, PathfindThenFollowPath
from phoenix6 import swerve
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Rotation2d, Translation2d, Transform2d
from wpimath.units import rotationsToRadians, degrees, radians, degreesToRadians, radiansToDegrees, metersToInches, inchesToMeters
import math
from subsystems import limelight
from commands.odometry_fuse import VisOdoFuseCommand
from commands.odometry_snap2Line import SnapToLineCommand

from constants import RobotDimensions



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
        
        # drive to a specific orientation relative to a target
        # TODO: replace with command import : swerve.requests.DriveAtTargetCommand
        # TODO: create error terms for vision data and target location
        # TODO: implement a PID controller to drive to the target
        # self._driveTargetRelative = (swerve.requests.RobotCentric()
        #                         .with_velocity_x(.1)
        #                         .with_velocity_y(0.2)
        #                         .with_rotational_rate(0.3))

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Red2-Algae")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)
        
        
        # PathfindThenFollowPath()
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
                        -controlAugment.smooth(self._joystick.getLeftY()) * self._max_speed
                        * self.invertBlueRedDrive 
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -controlAugment.smooth(self._joystick.getLeftX()) * self._max_speed
                        * self.invertBlueRedDrive 
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -controlAugment.smooth(self._joystick.getRightX()) * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                    .with_center_of_rotation(Translation2d(x= self.robotWidthBumpered*(controlAugment
                                                                                    .smooth(controlAugment
                                                                                            .one_side_control_only( self._joystick.getRightY(), 'Pos'))),
                                                            # want y translation to depend on direction of turn
                                                            y= math.copysign(1,self._joystick.getRightX())) *
                                                                self.robotWidthBumpered*(controlAugment
                                                                                            .smooth(controlAugment
                                                                                                    .one_side_control_only( self._joystick.getRightY(), 'Pos'))))
                    # shift the center of rotation to opposite front corner, if the driver pulls down on the right stick in addition to the side. 
                    # This should allow some nice defensive roll-off maneuvers                        
                )
            )
        )
        
        #section vision related commands
        #take in vision data and update the odometery... there has to be a better way in crte code...
        self._joystick.y().negate().whileTrue( self.vis_odo_fuse_command.alongWith(commands2.PrintCommand("commanded to try VISION update. \nq\nqqq\nqqqqqqq\nqqqqqqqqqqqqqqqqqqqqqqqqqqqq\nqqqqqqq\n---\n"))  )
        #Focus in on the target and move relative to it
        # self._joystick.rightStick().whileTrue(
        #     self.drivetrain.apply_request(lambda: self._driveTargetRelative) #might work until need dynamic values
        # )
        self._joystick.x().onTrue(SnapToLineCommand(self.drivetrain))


        #endsection vision related commands

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY() * self.invertBlueRedDrive , 
                               -self._joystick.getLeftX() * self.invertBlueRedDrive )
                )
            )
        )

        # self._joystick.pov(0).whileTrue(
        #     self.drivetrain.apply_request(
        #         lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0)
        #     )
        # )
        #trim out the gyro drift; if press POV 0 and move right stick update the drivetrain rotation, but conditional that the right stick input is more than .1
        self._joystick.pov(0).whileTrue(
            commands2.ConditionalCommand(
                self.drivetrain.apply_request(
                    lambda: self.drivetrain.reset_rotation(self.drivetrain.get_rotation3d().toRotation2d() + Rotation2d(0,0,degreesToRadians( -self._joystick.getRightX())))
                    )
                ,
                self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0)
                )
                ,
                lambda: self._joystick.getRightX().__abs__() > 0.1,
            )
        )
        self._joystick.pov(180).whileTrue(
            commands2.ConditionalCommand(
            self.drivetrain.apply_request(
                lambda: self.drivetrain.reset_rotation(self.drivetrain.get_rotation3d().toRotation2d() + Rotation2d(0,0,degreesToRadians( self._joystick.getRightX())))
            ),
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(-0.5).with_velocity_y(0)
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

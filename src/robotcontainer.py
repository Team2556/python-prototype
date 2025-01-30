#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from subsystems import algae

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        
        self.algae = algae.AlgaeHandler()

        self._logger = Telemetry(self._max_speed)

        # This one's probably used for moving
        self._joystick = commands2.button.CommandXboxController(0)
        # This one's probably used for scoring stuff
        self._joystick2 = commands2.button.CommandXboxController(1)

        self.drivetrain = TunerConstants.create_drivetrain()

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        def curve_off_input(value: float, blend_factor: float = 0.60) -> float:
            """The bend_factor is the factor that determines how much the curve is applied.
            The higher the bend_factor, the more the curve is applied, and thus more 'deadband' is added.
            """
            value_update = blend_factor * value ** 9 + (1 - blend_factor) * value
            #limit value to -1 to 1
            if value_update > 1:
                return 1
            elif value_update < -1:
                return -1
            else:
                return value_update

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -curve_off_input(self._joystick.getLeftY()) * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -curve_off_input(self._joystick.getLeftX()) * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -curve_off_input(self._joystick.getRightX(), 
                                         blend_factor=.7) * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )
        
        # Do the default command thing that tells algae controller stuff
        # Algae controls with controller 2 left joystick
        # It's negative because that's how xbox controllers work
        self.algae.setDefaultCommand(
            commands2.cmd.run(
                lambda: self.algae.cycle(self._joystick2.getRightY() * -1), self.algae
            )
        )

        # This commant seperates default commands from joystick inputs...

        self._joystick.a().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._brake
            )
        )
        
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
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
        return commands2.cmd.print_("No autonomous command configured")

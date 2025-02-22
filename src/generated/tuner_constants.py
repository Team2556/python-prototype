from phoenix6 import CANBus, configs, hardware, signals, swerve, units
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from wpimath.units import inchesToMeters
from networktables import NetworkTables


class TunerConstants:
    """
    Generated by the Tuner X Swerve Project Generator
    https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
    """

    # Both sets of gains need to be tuned to your individual robot

    # The steer motor uses any SwerveModule.SteerRequestType control request with the
    # output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    _steer_gains = (
        configs.Slot0Configs()
        .with_k_p(10) #was generated as (100)
        .with_k_i(0)
        .with_k_d(0.5)#was generated as (0.5)
        .with_k_s(0.1)#was generated as (0.1)
        .with_k_v(1.59)#was generated as (1.59)
        .with_k_a(0)
        .with_static_feedforward_sign(signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
    )
    # When using closed-loop control, the drive motor uses the control
    # output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    _drive_gains = (
        configs.Slot0Configs()
        .with_k_p(0.00442645)
        .with_k_i(0)
        .with_k_d(0)
        .with_k_s(0.07664325)
        .with_k_v(0.11421)
    )

    # The closed-loop output type to use for the steer motors;
    # This affects the PID/FF gains for the steer motors
    _steer_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE
    # The closed-loop output type to use for the drive motors;
    # This affects the PID/FF gains for the drive motors
    _drive_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE

    # The type of motor used for the drive motor
    _drive_motor_type = swerve.DriveMotorArrangement.TALON_FX_INTEGRATED
    # The type of motor used for the drive motor
    _steer_motor_type = swerve.SteerMotorArrangement.TALON_FX_INTEGRATED

    # The remote sensor feedback type to use for the steer motors;
    # When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    # _steer_feedback_type = swerve.SteerFeedbackType.FUSED_CANCODER #giving us errors
    _steer_feedback_type = swerve.SteerFeedbackType.REMOTE_CANCODER 

    # The stator current at which the wheels start to slip;
    # This needs to be tuned to your individual robot #TODO: #12 _slip_current needs to be tuned
    splip_curremt_degrade = 1
    _slip_current: units.ampere = 120.0 / splip_curremt_degrade

    # Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    # Some configs will be overwritten; check the `with_*_initial_configs()` API documentation.
    _drive_initial_configs = configs.TalonFXConfiguration()
    _steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs()
        # Swerve azimuth does not require much torque output, so we can set a relatively low
        # stator current limit to help avoid brownouts without impacting performance.
        .with_stator_current_limit(30).with_stator_current_limit_enable(True)
    )
    _encoder_initial_configs = configs.CANcoderConfiguration()
    # Configs for the Pigeon 2; leave this None to skip applying Pigeon 2 configs
    _pigeon_configs: configs.Pigeon2Configuration | None = None

    # CAN bus that the devices are located on;
    # All swerve devices must share the same CAN bus
    canbus = CANBus("", "./logs/example.hoot")

    # Theoretical free speed (m/s) at 12 V applied output;
    # This needs to be tuned to your individual robot
    speed_at_12_volts: units.meters_per_second = 2.73 # TODO: #10 Change this to the actual speed 
    #getting 34.5 rotations/sec at 4 volts .'. 34.5 * 2 * pi * 0.0508 = 10.9 m/s
    

    # Every 1 rotation of the azimuth results in _couple_ratio drive motor turns;
    # This may need to be tuned to your individual robot
    # Protobot: turned in steer direction 4 times, drive motor turned 2 plus 7 out of the 43 drive bevel teeth
    _couple_ratio = (2 + 7/43) / 4 # generated: 3.5714285714285716

    _drive_gear_ratio = 6.746031746031747
    _steer_gear_ratio = 12.8
    _wheel_radius: units.meter = inchesToMeters(2)

    _invert_left_side = False
    _invert_right_side = True

    _pigeon_id = 13

    # These are only used for simulation
    _steer_inertia: units.kilogram_square_meter = 0.01
    _drive_inertia: units.kilogram_square_meter = 0.01
    # Simulated voltage necessary to overcome friction
    _steer_friction_voltage: units.volt = 0.2
    _drive_friction_voltage: units.volt = 0.2

    drivetrain_constants = (
        swerve.SwerveDrivetrainConstants()
        .with_can_bus_name(canbus.name)
        .with_pigeon2_id(_pigeon_id)
        .with_pigeon2_configs(_pigeon_configs)
    )

    _constants_creator: swerve.SwerveModuleConstantsFactory[configs.TalonFXConfiguration, configs.TalonFXConfiguration, configs.CANcoderConfiguration] = (
        swerve.SwerveModuleConstantsFactory()
        .with_drive_motor_gear_ratio(_drive_gear_ratio)
        .with_steer_motor_gear_ratio(_steer_gear_ratio)
        .with_coupling_gear_ratio(_couple_ratio)
        .with_wheel_radius(_wheel_radius)
        .with_steer_motor_gains(_steer_gains)
        .with_drive_motor_gains(_drive_gains)
        .with_steer_motor_closed_loop_output(_steer_closed_loop_output)
        .with_drive_motor_closed_loop_output(_drive_closed_loop_output)
        .with_slip_current(_slip_current)
        .with_speed_at12_volts(speed_at_12_volts)
        .with_drive_motor_type(_drive_motor_type)
        .with_steer_motor_type(_steer_motor_type)
        .with_feedback_source(_steer_feedback_type)
        .with_drive_motor_initial_configs(_drive_initial_configs)
        .with_steer_motor_initial_configs(_steer_initial_configs)
        .with_encoder_initial_configs(_encoder_initial_configs)
        .with_steer_inertia(_steer_inertia)
        .with_drive_inertia(_drive_inertia)
        .with_steer_friction_voltage(_steer_friction_voltage)
        .with_drive_friction_voltage(_drive_friction_voltage)
    )


    # Front Left
    _front_left_drive_motor_id = 10
    _front_left_steer_motor_id = 11
    _front_left_encoder_id = 12
    _front_left_encoder_offset: units.rotation = 0.453125
    _front_left_steer_motor_inverted = False
    _front_left_encoder_inverted = False

    _front_left_x_pos: units.meter = inchesToMeters(11.75)
    _front_left_y_pos: units.meter = inchesToMeters(11.75)

    # Front Right
    _front_right_drive_motor_id = 7
    _front_right_steer_motor_id = 8
    _front_right_encoder_id = 9
    _front_right_encoder_offset: units.rotation = 0.439453125
    _front_right_steer_motor_inverted = False
    _front_right_encoder_inverted = False

    _front_right_x_pos: units.meter = inchesToMeters(11.75)
    _front_right_y_pos: units.meter = inchesToMeters(-11.75)

    # Back Left
    _back_left_drive_motor_id = 4
    _back_left_steer_motor_id = 5
    _back_left_encoder_id = 6
    _back_left_encoder_offset: units.rotation = 0.192626953125
    _back_left_steer_motor_inverted = False
    _back_left_encoder_inverted = False

    _back_left_x_pos: units.meter = inchesToMeters(-11.75)
    _back_left_y_pos: units.meter = inchesToMeters(11.75)

    # Back Right
    _back_right_drive_motor_id = 1
    _back_right_steer_motor_id = 2
    _back_right_encoder_id = 3
    _back_right_encoder_offset: units.rotation = 0.36279296875
    _back_right_steer_motor_inverted = False
    _back_right_encoder_inverted = False

    _back_right_x_pos: units.meter = inchesToMeters(-11.75)
    _back_right_y_pos: units.meter = inchesToMeters(-11.75)


    front_left = _constants_creator.create_module_constants(
        _front_left_steer_motor_id,
        _front_left_drive_motor_id,
        _front_left_encoder_id,
        _front_left_encoder_offset,
        _front_left_x_pos,
        _front_left_y_pos,
        _invert_left_side,
        _front_left_steer_motor_inverted,
        _front_left_encoder_inverted,
    )
    front_right = _constants_creator.create_module_constants(
        _front_right_steer_motor_id,
        _front_right_drive_motor_id,
        _front_right_encoder_id,
        _front_right_encoder_offset,
        _front_right_x_pos,
        _front_right_y_pos,
        _invert_right_side,
        _front_right_steer_motor_inverted,
        _front_right_encoder_inverted,
    )
    back_left = _constants_creator.create_module_constants(
        _back_left_steer_motor_id,
        _back_left_drive_motor_id,
        _back_left_encoder_id,
        _back_left_encoder_offset,
        _back_left_x_pos,
        _back_left_y_pos,
        _invert_left_side,
        _back_left_steer_motor_inverted,
        _back_left_encoder_inverted,
    )
    back_right = _constants_creator.create_module_constants(
        _back_right_steer_motor_id,
        _back_right_drive_motor_id,
        _back_right_encoder_id,
        _back_right_encoder_offset,
        _back_right_x_pos,
        _back_right_y_pos,
        _invert_right_side,
        _back_right_steer_motor_inverted,
        _back_right_encoder_inverted,
    )

    @classmethod
    def create_drivetrain(clazz) -> CommandSwerveDrivetrain:
        """
        Creates a CommandSwerveDrivetrain instance.
        This should only be called once in your robot program.
        """
        return CommandSwerveDrivetrain(
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            clazz.drivetrain_constants,
            [
                clazz.front_left,
                clazz.front_right,
                clazz.back_left,
                clazz.back_right,
            ],
        )

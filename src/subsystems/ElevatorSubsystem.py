import commands2
import wpilib
import wpimath.controller
from wpimath.controller import PIDController, ProfiledPIDController
import wpimath.trajectory
import phoenix6
from phoenix6 import hardware, controls, configs, StatusCode, signals
from phoenix6.controls import Follower
from phoenix6.signals import NeutralModeValue
from constants import ElevatorConstants
from math import pi


# Create a new ElevatorSubsystem -- using phonix6 pid
class ElevatorSubsystem(commands2.Subsystem):# .ProfiledPIDSubsystem):
    def __init__(self) -> None:
        '''IM AN ELEVATOR'''

        super().__init__( )
        # self.elevcontroller = self.getController# .controler# wpimath.controller.ProfiledPIDController(5.0, 0, 0)

        # Start at position 0, use slot 0
        wpilib.SmartDashboard.putNumber("Elevator/Setpoint", 0.0)
        self.setpoint = wpilib.SmartDashboard.getNumber("Elevator/Setpoint", 0.0)

        self.elevmotor_right = phoenix6.hardware.TalonFX(ElevatorConstants.kRightMotorPort, "rio")
        self.elevmotor_left = phoenix6.hardware.TalonFX( ElevatorConstants.kLeftMotorPort, "rio")
        self.elevmotor_right.set_control(request=Follower(self.elevmotor_left.device_id, oppose_master_direction=True))
        self.elevmotor_right.setNeutralMode(NeutralModeValue.BRAKE)
        self.elevmotor_left.setNeutralMode(NeutralModeValue.BRAKE)
        # self.control = controls.DutyCycleOut(0)
        self.elevCANcoder_left = phoenix6.hardware.CANcoder(ElevatorConstants.kLeftMotorPort)
        self.elevCANcoder_right = phoenix6.hardware.CANcoder(ElevatorConstants.kRightMotorPort)


        cfg = configs.TalonFXConfiguration()
        cfg.slot0.k_p = ElevatorConstants.kElevatorKp
        cfg.slot0.k_i = ElevatorConstants.kElevatorKi
        cfg.slot0.k_d = ElevatorConstants.kElevatorKd
        
        cfg.slot0.integralZone = 0
        cfg.slot0.forwardSoftLimitThreshold = 0

        cfg.slot0.gravity_type = signals.GravityTypeValue.ELEVATOR_STATIC
        cfg.slot0.static_feedforward_sign = signals.StaticFeedforwardSignValue.USE_VELOCITY_SIGN
        cfg.slot0.k_g = ElevatorConstants.kGVolts
        

        cfg.slot0.k_a = ElevatorConstants.kAVoltSecondSquaredPerMeter
        cfg.slot0.k_v = ElevatorConstants.kVVoltSecondPerMeter
        # cfg.slot0.maxIntegralAccumulator = 0
        

        # cfg.voltage.peak_output_forward = 8
        cfg.stator_current_limit_enable = True
        cfg.torque_current.peak_forward_torque_current = ElevatorConstants.kpeak_forward_torque_current
        cfg.torque_current.peak_reverse_torque_current = ElevatorConstants.kpeak_reverse_torque_current
        elevmotorLimitswitch_cfg = (configs.HardwareLimitSwitchConfigs()
                                       .with_forward_limit_enable(True)
                                       .with_forward_limit_autoset_position_enable(True)
                                       .with_forward_limit_autoset_position_value(self.distanceToRotations(ElevatorConstants.kMaxElevatorHeight))
                                       .with_forward_limit_remote_sensor_id(ElevatorConstants.kTopLimitSwitchChannel)
                                       .with_forward_limit_type(signals.ForwardLimitTypeValue.NORMALLY_OPEN)
                                       
                                       .with_reverse_limit_autoset_position_enable(True)
                                       .with_reverse_limit_autoset_position_value(self.distanceToRotations(ElevatorConstants.kElevatorOffsetMeters))
                                       .with_reverse_limit_remote_sensor_id(ElevatorConstants.kBottomLimitSwitchChannel)
                                       .with_reverse_limit_type(signals.ReverseLimitTypeValue.NORMALLY_OPEN)
                                       )
        cfg.with_hardware_limit_switch(elevmotorLimitswitch_cfg)
        
        elevmotorFeedback_cfg = (configs.FeedbackConfigs().with_feedback_sensor_source(signals.FeedbackSensorSourceValue.ROTOR_SENSOR)
                                 .with_sensor_to_mechanism_ratio(ElevatorConstants.kElevatorGearing)#/(2*pi*ElevatorConstants.kElevatorDrumRadius))
                                 )
        cfg.with_feedback(elevmotorFeedback_cfg)

        self.cfg_slot0 = cfg.slot0

        #. wpilib.DigitalInput(ElevatorConstants.kTopLimitSwitchChannel)
        # bottomelevmotorlimitswitch_cfg = wpilib.DigitalInput(ElevatorConstants.kBottomLimitSwitchChannel)
        # (cfg.slot0
        #  .with_stator_current_limit_enable(True)
        #  .with_stator_current_limit(20))
        
        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.elevmotor_left.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        print(f"Configured TalonFX {ElevatorConstants.kLeftMotorPort} with status: {status.name}\n {cfg.slot0}^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")

        #creat handel for the control
        self.position_voltage = controls.PositionVoltage(0).with_slot(0)
        # Make sure we start at 0
        self.elevmotor_left.set_position(self.distanceToRotations(ElevatorConstants.kElevatorOffsetMeters))
        # self.elevmotor_left.
        #move up some
        # self.elevmotor_left.set_control(request=self.position_voltage.with_position(.25))


        
        # # self.joystick2 = commands2.button.CommandXboxController(ElevatorConstants.kJoystickPort)
        # self.feedforward = wpimath.controller.ElevatorFeedforward(
        #     ElevatorConstants.kSVolts,
        #     ElevatorConstants.kGVolts,
        #     ElevatorConstants.kVVoltSecondPerMeter,
        #     ElevatorConstants.kAVoltSecondSquaredPerMeter,
        # )

        # # Start elevator at rest in neutral position
        # self.setGoal(ElevatorConstants.kElevatorOffsetMeters)

    def updateSlot0(self,  k_p: float = None, k_i:float =None, k_d:float=None, k_g: float=None   ) -> None:
        updated = False

        if self.cfg_slot0.k_p != k_p: 
            self.cfg_slot0.k_p = k_p
            updated = True
        if  self.cfg_slot0.k_i != k_i: 
            self.cfg_slot0.k_i = k_i
            updated = True
        if  self.cfg_slot0.k_d != k_d: 
            self.cfg_slot0.k_d = k_d
            updated = True
        if  self.cfg_slot0.k_g != k_g: 
            self.cfg_slot0.k_g = k_g
            updated = True
        #TODO: add others, if needed
        if updated:
            status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
            for _ in range(0, 5):
                status = self.elevmotor_left.configurator.apply(self.cfg_slot0 )
                if status.is_ok():
                    break
            if not status.is_ok():
                print(f"Could not apply updated gravity compensation, error code: {status.name}")
            else:
                print(f"Updated slot0 {self.cfg_slot0} with status: {status.name} /n  Get rid of this update for competition")

    def distanceToRotations(self, distance: float) -> float:
        return distance * ElevatorConstants.kElevatorGearing/(2*pi*ElevatorConstants.kElevatorDrumRadius)
    def rotationsToDistance(self, rotations: float) -> float:
        return rotations * 2*pi*ElevatorConstants.kElevatorDrumRadius/ElevatorConstants.kElevatorGearing
    
    def update_setpoint(self, setpoint: float, incremental = True, constrain: bool = True) -> None:
        '''Setpoint is in meters'''
        if incremental:
            self.setpoint += setpoint
        else:
            self.setpoint = setpoint
        #within bounds check and reset to bounds:
        if constrain:
            if self.setpoint > ElevatorConstants.kMaxElevatorHeight:
                self.setpoint = ElevatorConstants.kMaxElevatorHeight
            elif self.setpoint < ElevatorConstants.kMinElevatorHeight:
                self.setpoint = ElevatorConstants.kMinElevatorHeight
        wpilib.SmartDashboard.putNumber("Elevator/Setpoint", self.setpoint)

    def moveElevator(self, meters=None) -> None:
        if not meters:
            meters = self.setpoint
        self.elevmotor_left.set_control(self.position_voltage.with_position(self.distanceToRotations(meters)))
        self.elevmotor_left.set_control(self.position_voltage.   with_position(self.distanceToRotations(meters)))
        
    def periodic(self):
        wpilib.SmartDashboard.putNumber("Elevator/Position_calced", self.rotationsToDistance(self.elevmotor_left.get_position().value))
        wpilib.SmartDashboard.putNumber("Elevator/SetpointError_calced", self.setpoint - self.rotationsToDistance(self.elevmotor_left.get_position().value))
        
        return super().periodic()
    
    # def _useOutput(
    #     self, output: float, setpoint: wpimath.trajectory.TrapezoidProfile.State
    # ) -> None:
    #     # Calculate the feedforward from the setpoint
    #     feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)

    #     # Add the feedforward to the PID output to get the motor output
    #     self.elevmotors.setVoltage(output + feedforward)
    # def _getMeasurement(self) -> float:
    #     return self.elevmotors.set_position() + ElevatorConstants.kElevatorOffsetMeters
        
    # def disablePIDSubsystems(self) -> None:
    #     """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
    #     This should be called on robot disable to prevent integral windup."""
    #     self.disable()



    # def elevatorPeriodic(self) -> None:

    #     if self.joystick2.rightTrigger():
    #         # Here, we run PID control like normal, with a constant setpoint of 30in (0.762 meters).
    #         pidOutput = self.elevcontroller.calculate(self.elevmotors.set_position(), 1.27)
    #         self.elevmotors.setVoltage(pidOutput)
    #     elif self.joystick2.leftTrigger():
    #         pidOutput = self.elevcontroller.calculate(self.elevmotors.set_position(), 0.762)
    #         self.elevmotors.setVoltage(pidOutput)
    #     elif self.joystick2.leftBumper():
    #         pidOutput = self.elevcontroller.calculate(self.elevmotors.set_position(), 0.3)
    #         self.elevmotors.setVoltage(pidOutput)
    #     elif self.joystick2.rightBumper():
    #         pidOutput = self.elevcontroller.calculate(self.elevmotors.set_position(), 0)
    #         self.elevmotors.setVoltage(pidOutput)
    #     else:
    #         # Otherwise we disable the motor
    #         self.elevmotors.set(0.0)


    #     if self.topelevmotorlimitswitch.get():
    #         self.moveElevator(meters = self.getMeasurement - ElevatorConstants.kElevatorDistanceMovedAfterContactWithLimitSwitch)
    #     elif self.bottomelevmotorlimitswitch.get():
    #         self.moveElevator(meters = self.getMeasurement + ElevatorConstants.kElevatorDistanceMovedAfterContactWithLimitSwitch)
            
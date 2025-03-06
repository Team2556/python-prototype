# Thing that controls the algae

from wpilib import SmartDashboard, DigitalInput
from constants import AlgaeConstants

from commands2.subsystem import Subsystem

import phoenix6

class AlgaeSubsystem(Subsystem):
    '''This thing does algae intake and discharge.'''
    
    def __init__(self):
        # Declare motor controllers
        self.pivotMotor = phoenix6.hardware.TalonFX(AlgaeConstants.kPivotMotorChannel, "rio")
        self.intakeMotor = phoenix6.hardware.TalonFX(AlgaeConstants.kIntakeWheelsChannel, "rio")
        # Declare limit switch
        self.limitSwitch = DigitalInput(AlgaeConstants.kLimitSwitchChannel)
        # self.otherLimitSwitch = DigitalInput(AlgaeConstants.kOtherLimitSwitchChannel) 
        # There might be another limit switch maybe
        
        # Make it so if motor is set at 0 then it resists any turning if force is applied to it
        # So algae doesn't fall out and stuff
        self.pivotMotor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.intakeMotor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        
        # Creates a "CTRE Control Request Object" for making it spin
        self.velocityOut = phoenix6.controls.VoltageOut(output=0)
        
        # All the following config stuff so setting position PID still works
        cfg = phoenix6.configs.TalonFXConfiguration()
        # TODO: make these constants
        cfg.slot0.k_p = 0.1
        cfg.slot0.k_i = 0
        cfg.slot0.k_d = 0
        
        cfg.slot0.integralZone = 0
        cfg.slot0.forwardSoftLimitThreshold = 0

        # TODO: adding this stuff might make it more accurate/faster
        # cfg.slot0.gravity_type = phoenix6.signals.GravityTypeValue.ARM_COSINE
        # cfg.slot0.static_feedforward_sign = phoenix6.signals.StaticFeedforwardSignValue.USE_VELOCITY_SIGN
        # cfg.slot0.k_g = ElevatorConstants.kGVolts
        
        # cfg.slot0.k_a = ElevatorConstants.kAVoltSecondSquaredPerMeter
        # cfg.slot0.k_v = ElevatorConstants.kVVoltSecondPerMeter
        # cfg.slot0.maxIntegralAccumulator = 0
        
        # # cfg.voltage.peak_output_forward = 8
        # cfg.stator_current_limit_enable = True
        
        cfg.torque_current.peak_forward_torque_current = AlgaeConstants.kPeakForwardTorqueCurrent
        cfg.torque_current.peak_reverse_torque_current = AlgaeConstants.kPeakReverseTorqueCurrent
        
        # Actually give the motor all the config stuff
        self.PIDconfig = cfg
        self.pivotMotor.configurator.apply(self.PIDconfig)
        
        # Creates another "CTRE Control Request Object" for making it position
        self.positionVoltage = phoenix6.controls.PositionVoltage(
            velocity=AlgaeConstants.kPivotRotationsPerSecond,
            limit_forward_motion=True,
            limit_reverse_motion=True,
            ignore_hardware_limits=True
        ).with_slot(0)#.e
        # How would I get the value of the bottom limit switch pls
        
    # I might need this for PID
    # def updateSlot0(self,  k_p: float = None, k_i:float =None, k_d:float=None, k_g: float=None   ) -> None:
    #     updated = False

    #     if self.cfg_slot0.k_p != k_p: 
    #         self.cfg_slot0.k_p = k_p
    #         updated = True
    #     if self.cfg_slot0.k_i != k_i: 
    #         self.cfg_slot0.k_i = k_i
    #         updated = True
    #     if self.cfg_slot0.k_d != k_d: 
    #         self.cfg_slot0.k_d = k_d
    #         updated = True
    #     if self.cfg_slot0.k_g != k_g: 
    #         self.cfg_slot0.k_g = k_g
    #         updated = True
    #     #TODO: add others, if needed
    #     if updated:
    #         #repeat up to 5 times
    #         status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
    #         for _ in range(0, 5):
    #             status = self.elevmotor_left.configurator.apply(self.cfg_slot0 )
    #             if status.is_ok():
    #                 break
    #         if not status.is_ok():
    #             print(f"Could not apply updated gravity compensation, error code: {status.name}")
    
    # Motor spinning functions
    def changePosition(self, position):
        '''Sets the position of the pivot motor (obvoiusly)'''
        # TODO: Actually test this
        self.pivotMotor.set_control(self.positionVoltage.with_position(position))
        # self.pivotMotor.set_position()
    
    def spinIntakeMotor(self, speed) -> None:
        '''Spins the intake motor (speed=1 is intake, speed=-1 is discharge hopefully)'''
        # TODO: Figure out the correct way to do this
        voltage = 12
        self.intakeMotor.set_control(self.velocityOut.with_output(speed * voltage))
        # self.intakeMotor.setVoltage(speed * voltage)
        # self.intakeMotor.set(speed)
        
    def getCurrent(self) -> bool:
        '''Gets the current of the intaking motor. 
            Useful for stopping the motor if it's trying to intake an algae further than it can'''  
        # TODO: Figure out if this is actually the correct function to use
        self.intakeMotor.get_supply_current().value
        
    def periodic(self) -> None:
        ...
    
    def getLimitSwitchActive(self, toggleLimitSwitch=True) -> bool:
        '''Returns true if limit switch is active (and toggleLimitSwitch is also true)'''
        return self.limitSwitch.get() and toggleLimitSwitch
        
    def updateSmartDashboard(self):
        SmartDashboard.putString("Algae/Pivot Position", self.pivotMotor.get_rotor_position().__str__())
        SmartDashboard.putString("Algae/Intake Speed", self.intakeMotor.get_motor_voltage().__str__())
        SmartDashboard.putBoolean("Algae/Limit Switch", self.limitSwitch.get())
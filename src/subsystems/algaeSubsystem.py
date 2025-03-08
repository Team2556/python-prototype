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
        
        self.setupSmartDashboard()
        
        # Creates a "CTRE Control Request Object" for making it spin
        self.velocityOut = phoenix6.controls.VoltageOut(output=0)
        
        # All the following config stuff so setting position PID still works
        cfg = phoenix6.configs.TalonFXConfiguration()
        # TODO: make these constants
        cfg.slot0.k_p = 0.1
        cfg.slot0.k_i = 0
        cfg.slot0.k_d = 0
        
        # Don't think these do anything at all
        # cfg.slot0.integralZone = 0
        # cfg.slot0.forwardSoftLimitThreshold = 0

        # TODO: adding this stuff might make it more accurate/faster/better/but it takes more time so maybe
        # cfg.slot0.gravity_type = phoenix6.signals.GravityTypeValue.ARM_COSINE
        # cfg.slot0.static_feedforward_sign = phoenix6.signals.StaticFeedforwardSignValue.USE_VELOCITY_SIGN
        # cfg.slot0.k_g = ElevatorConstants.kGVolts
        
        # cfg.slot0.k_a = ElevatorConstants.kAVoltSecondSquaredPerMeter
        # cfg.slot0.k_v = ElevatorConstants.kVVoltSecondPerMeter
        # cfg.slot0.maxIntegralAccumulator = 0
        
        # # cfg.voltage.peak_output_forward = 8
        # cfg.stator_current_limit_enable = True
        
        # Limits something on the motor so it's useful
        cfg.torque_current.peak_forward_torque_current = AlgaeConstants.kPeakForwardTorqueCurrent
        cfg.torque_current.peak_reverse_torque_current = AlgaeConstants.kPeakReverseTorqueCurrent
        
        # Actually give the motor all the config stuff
        self.PIDconfig = cfg
        self.pivotMotor.configurator.apply(self.PIDconfig)
        
        # Creates another "CTRE Control Request Object" for making it position
        self.positionVoltage = phoenix6.controls.PositionVoltage(
            0, # Position will be changed when this is actually used
            velocity=AlgaeConstants.kPivotRotationsPerSecond,
            limit_forward_motion=True,
            limit_reverse_motion=True,
            ignore_hardware_limits=True
        ).with_slot(0)#._.
        
    # I might need this for PID
    def updatePIDvalues(self, k_p: float = None, k_i: float = None, k_d : float = None, k_g: float = None) -> None:
        valueUpdated = False
        if self.PIDconfig.slot0.k_p != k_p: 
            self.PIDconfig.slot0.k_p = k_p
            valueUpdated = True
        if self.PIDconfig.slot0.k_i != k_i:  
            self.PIDconfig.slot0.k_i = k_i
            valueUpdated = True
        if self.PIDconfig.slot0.k_d != k_d: 
            self.PIDconfig.slot0.k_d = k_d
            valueUpdated = True
        if self.PIDconfig.slot0.k_g != k_g: 
            self.PIDconfig.slot0.k_g = k_g
            valueUpdated = True
        # TODO: Add other values here if needed (like k_f or something)
        if valueUpdated:
            # Repeat up to 5 times because it might not work some of the time
            status: phoenix6.StatusCode = phoenix6.StatusCode.STATUS_CODE_NOT_INITIALIZED
            for _ in range(0, 5):
                status = self.pivotMotor.configurator.apply(self.PIDconfig)
                if status.is_ok():
                    break
            if not status.is_ok(): 
                print(f"Could not apply updated gravity compensation, error code: {status.name}")
    
    def updatePivotSetpoint(self, setpoint: float, increment = False, constrain: bool = True) -> None:
        '''Setpoint is in meters of elevator elevation from lowest physical limit'''
        if increment:
            self.setpoint += setpoint
        else:
            self.setpoint = setpoint
        # Another restrain in case it's told to go past it's bounds
        if constrain:
            if self.setpoint > AlgaeConstants.kPivotMaxHeight:
                self.setpoint = AlgaeConstants.kPivotMaxHeight
            elif self.setpoint < AlgaeConstants.kPivotMinHeight:
                self.setpoint = AlgaeConstants.kPivotMinHeight
    
    # Motor spinning functions
    def changePivotPosition(self):
        '''Sets the position of the pivot motor (obvoiusly)'''
        # TODO: Actually test this
        self.pivotMotor.set_control(self.positionVoltage.with_position(self.setpoint)) 
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
    
    def getLimitSwitchActive(self, toggleLimitSwitch=True) -> bool:
        '''Returns true if limit switch is active (and toggleLimitSwitch is also true)'''
        return self.limitSwitch.get() and toggleLimitSwitch
        
    # TODO: New commands system probably has different tuning values
    # TODO: Add a cool list/grid layout with the new values to ShuffleBoard
    def setupSmartDashboard(self):
        '''Sets up all the SmartDashboard valuess'''
        SmartDashboard.putNumber("Algae/Pivot Time", AlgaeConstants.kPivotRotationsPerSecond)
        SmartDashboard.putNumber("Algae/Intake Multiplier", AlgaeConstants.kIntakeMultiplier)
        SmartDashboard.putBoolean("Algae/Toggle Limit Switch", AlgaeConstants.kToggleLimitSwitch)
        SmartDashboard.putNumber("Algae/Processing Delay", AlgaeConstants.kSpinProcessDelayValue)
        SmartDashboard.putNumber("Algae/Pivot Intake Position", AlgaeConstants.kPivotReefIntakingValue)
        SmartDashboard.putNumber("Algae/Pivot Processing Position", AlgaeConstants.kPivotProcessingValue)
        SmartDashboard.putNumber("Algae/Pivot Idle Position", AlgaeConstants.kPivotIdleValue)
        SmartDashboard.putNumber("Algae/Elevator Processing Position", AlgaeConstants.kElevatorProcessingPositionValue)
        SmartDashboard.putNumber("Algae/Elevator L2 Intaking Position", AlgaeConstants.kElevatorL2IntakePositionValue)
        SmartDashboard.putNumber("Algae/Elevator L3 Intaking Position", AlgaeConstants.kElevatorL3IntakePositionValue)
        SmartDashboard.putNumber("Algae/Pivot Intake Delay", AlgaeConstants.kIntakeDelayValue)
        SmartDashboard.putNumber("Algae/Pivot Process Delay", AlgaeConstants.kProcessDelayValue)
        
    def updateSmartDashboard(self):
        '''Put all SmartDashboard stuff for this subsystem here'''
        self.updatePIDvalues(
            SmartDashboard.getNumber("Algae/k_p"),
            SmartDashboard.getNumber("Algae/k_i"),
            SmartDashboard.getNumber("Algae/k_d"),
            SmartDashboard.getNumber("Algae/k_g"),
        )
        # Output values
        SmartDashboard.putNumber("Algae/Processing Delay Timer", round(self.processDelayTimer.get(), 2))
        SmartDashboard.putString("Algae/Pivot Waiting Position", self.pivotWaitingPosition)
        SmartDashboard.putNumber("Algae/Pivot Delay Timer", round(self.pivotDelayTimer.get(), 2))
        SmartDashboard.putString("Algae/Pivot Instruction", self.algaePivotPosition)
        SmartDashboard.putString("Algae/Pivot Position", self.pivotMotor.get_rotor_position().__str__())
        SmartDashboard.putString("Algae/Intake Speed", self.intakeMotor.get_motor_voltage().__str__())
        SmartDashboard.putBoolean("Algae/Limit Switch", self.limitSwitch.get())
        
        # Tuning values
        AlgaeConstants.kPivotRotationsPerSecond = SmartDashboard.getNumber("Algae/Pivot Time", AlgaeConstants.kPivotRotationsPerSecond)
        AlgaeConstants.kIntakeMultiplier = SmartDashboard.getNumber("Algae/Intake Multiplier", AlgaeConstants.kIntakeMultiplier)
        AlgaeConstants.kToggleLimitSwitch = SmartDashboard.getBoolean("Algae/Toggle Limit Switch", AlgaeConstants.kToggleLimitSwitch)
        AlgaeConstants.kSpinProcessDelayValue = SmartDashboard.getNumber("Algae/Processing Delay", AlgaeConstants.kSpinProcessDelayValue)
        AlgaeConstants.kPivotReefIntakingValue = SmartDashboard.getNumber("Algae/Pivot Intake Position", AlgaeConstants.kPivotReefIntakingValue)
        AlgaeConstants.kPivotProcessingValue = SmartDashboard.getNumber("Algae/Pivot Processing Position", AlgaeConstants.kPivotProcessingValue)
        AlgaeConstants.kPivotIdleValue = SmartDashboard.getNumber("Algae/Pivot Idle Position", AlgaeConstants.kPivotIdleValue)
        AlgaeConstants.kElevatorProcessingPositionValue = SmartDashboard.getNumber("Algae/Elevator Processing Position", AlgaeConstants.kElevatorProcessingPositionValue)
        AlgaeConstants.kElevatorL2IntakePositionValue = SmartDashboard.getNumber("Algae/Elevator L2 Intaking Position", AlgaeConstants.kElevatorL2IntakePositionValue)
        AlgaeConstants.kElevatorL3IntakePositionValue = SmartDashboard.getNumber("Algae/Elevator L3 Intaking Position", AlgaeConstants.kElevatorL3IntakePositionValue)
        AlgaeConstants.kIntakeDelayValue = SmartDashboard.putNumber("Algae/Pivot Intake Delay", AlgaeConstants.kIntakeDelayValue)
        AlgaeConstants.kProcessDelayValue = SmartDashboard.putNumber("Algae/Pivot Process Delay", AlgaeConstants.kProcessDelayValue)
    
    def periodic(self) -> None:
        # Sets setpoint to 0 if bottom limit switch active. No top limit switch though so umm...
        if self.limitSwitch.get():
            self.setpoint = 0
        # Stop intake motor if motor current supply value says to stop
        if self.getCurrent() > AlgaeConstants.kAmpValueToDetectIfMotorStalled:
            self.spinIntakeMotor(0)
        # Updates SmartDashboard
        self.updateSmartDashboard()
# Thing that controls the algae

import wpilib
from wpilib import SmartDashboard
from constants import AlgaeConstants

from commands2.subsystem import Subsystem

import phoenix6

class AlgaeHandler(Subsystem):
    '''This thing does algae intake and discharge'''
    def __init__(self):
        # We don't know the channels yet
        motorChannel1 = AlgaeConstants.kIntakeCANAddress1
        motorChannel2 = AlgaeConstants.kIntakeCANAddress2
        limitSwitchChannel = AlgaeConstants.kAlgaeLimitSwitchChannel
        
        # 1 = The one spinning clockwise when intaking
        # 2 = The one spinning counterclockwise when intaking
        self.algaeMotor1 = phoenix6.hardware.TalonFX(motorChannel1, "rio")
        self.algaeMotor2 = phoenix6.hardware.TalonFX(motorChannel2, "rio")
        
        # Make it so if motor is set at 0 then it resists any turning if force is applied to it
        # So algae doesn't fall out
        self.algaeMotor1.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.algaeMotor2.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        
        # Make it so motor 2 spins opposite of motor 1
        self.algaeMotor2.set_control(
            request = phoenix6.controls.Follower(self.algaeMotor1.device_id, oppose_master_direction=True)
        )
        
        # Creates a "CTRE Control Request Object"
        self.velocityOut = phoenix6.controls.VoltageOut(output=0)
        
        # Declare limit switch
        self.limitSwitch = wpilib.DigitalInput(limitSwitchChannel)
                
        # Settings to tune
        self.deadband = 0.1
        self.motorSpeedMultiplier = 1 # Needs to be between -1 and 1
        
        # Testing variable that ignores limit switches if False (will always be True during games)
        self.toggleLimitSwitch = True
        
        # SmartDasboard setting
        self.toggleSmartDashboard = True
        if self.toggleSmartDashboard:
            self.setupSmartDashboard()
    
    def cycle(self, controllerRightYInput) -> None:
        '''Gets called periodically; updates the algae motors based on the controller'''
        
        # Change the input stuffs
        speed = self.curveOffInput(controllerRightYInput, self.deadband)
        # TODO: Change this to use the one main curveOffInput thing later
        
        # Account for limit switches (if limit switch active then you can't intake)
        if self.limitSwitch.get() and speed > 0 and self.toggleLimitSwitch:
            speed = 0
        
        self.spinMotors(speed)
        
        # Change SmartDashboard motor multiplier because it can't be more than one
        if self.toggleSmartDashboard and self.motorSpeedMultiplier > 1:
                SmartDashboard.putNumber("Motor Multiplier", 1)
        if self.toggleSmartDashboard and self.motorSpeedMultiplier < -1:
                SmartDashboard.putNumber("Motor Multiplier", -1)
        
        # SmartDasboard updating
        if self.toggleSmartDashboard:
            self.updateSmartDashboard()
    
    def curveOffInput(self, speed, deadband=0.1):
        '''Does stuff like the cubing and the deadband'''
        # Do the deadband
        if speed < deadband and speed > -deadband:
            return 0
        
        # Do the cubing
        speed = speed ** 3
        
        # Do the motor multiplier (from 1 to -1)
        if self.motorSpeedMultiplier > 1: self.motorSpeedMultiplier = 1
        if self.motorSpeedMultiplier < -1: self.motorSpeedMultiplier = -1
        
        speed *= self.motorSpeedMultiplier
        
        return speed
    
    def spinMotors(self, speed) -> None:
        '''Spins the motors (speed=1 is intake, speed=-1 is discharge hopefully)'''
        self.algaeMotor1.set_control(self.velocityOut.with_output(speed))
        
    def setupSmartDashboard(self) -> None:
        '''Sets up the Smart Dashboard for Algae with all the cool things'''
        SmartDashboard.putNumber("Motor1", self.algaeMotor1.get())
        SmartDashboard.putNumber("Motor2", self.algaeMotor2.get())
        SmartDashboard.putBoolean("Limit Switch", self.limitSwitch.get())
        
        SmartDashboard.putNumber("Deadband", self.deadband)
        SmartDashboard.putNumber("Motor Multiplier", self.motorSpeedMultiplier)
        SmartDashboard.putBoolean("Toggle Limit Switch", self.toggleLimitSwitch)
        
    def updateSmartDashboard(self) -> None:
        '''Updates the Smart Dashboard for Algae with all the cool things'''
        
        # Update values TO the Smart Dashboard
        SmartDashboard.putNumber("Motor1", self.algaeMotor1.get())
        SmartDashboard.putNumber("Motor2", self.algaeMotor2.get())
        SmartDashboard.putBoolean("Limit Switch", self.limitSwitch.get())
        
        # Update values FROM the Smart Dashboard
        self.deadband = SmartDashboard.getNumber("Deadband", self.deadband)
        self.motorSpeedMultiplier = SmartDashboard.getNumber("Motor Multiplier", self.motorSpeedMultiplier)
        self.toggleLimitSwitch = SmartDashboard.getBoolean("Toggle Limit Switch", self.toggleLimitSwitch)

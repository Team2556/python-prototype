# Thing that controls the algae

import wpilib
from wpilib import SmartDashboard
from constants import AlgaeConstants

from commands2.subsystem import Subsystem

class AlgaeHandler(Subsystem):
    '''This thing uses two inputs to intake or discharge'''
    def __init__(self):
        # We don't know the channels yet
        motorChannel1 = AlgaeConstants.kIntakeCANAddress1
        motorChannel2 = AlgaeConstants.kIntakeCANAddress2
        limitSwitchChannel = AlgaeConstants.kAlgaeLimitSwitchChannel
        
        # 1 = The one spinning clockwise when intaking
        # 2 = The one spinning counterclockwise when intaking
        self.algaeMotor1 = wpilib.PWMSparkMax(motorChannel1)
        self.algaeMotor2 = wpilib.PWMSparkMax(motorChannel2)
        self.limitSwitch = wpilib.DigitalInput(limitSwitchChannel)
        # set() function makes these turn
        
        # Records direction of the motors (1 is intake, -1 is discharge)
        self.spinningDirection = 0
        
        # Records when button is pressing so override system works
        self.intakeActive = False
        self.dischargeActive = False
        
        # Settings to tune
        self.deadband = 0.1
        self.motorMultiplier = 1
        self.toggleLimitSwitch = True
        
        # SmartDasboard setting
        self.toggleSmartDashboard = True
        if self.toggleSmartDashboard:
            self.setupSmartDashboard()
    
    def cycle(self, controllerLeftYInput) -> None:
        '''Gets called periodically; updates the algar motors based on the controller'''
        
        # Change the input stuffs
        dir = self.curveOffInput(controllerLeftYInput, self.deadband)
        
        # Account for limit switches (if limit switch active then you can't intake)
        if self.limitSwitch.get() and dir > 0 and self.toggleLimitSwitch:
            dir = 0
        
        self.spinMotors(dir)
        
        # Change SmartDashboard motor multiplier because it can't be more than one
        if self.toggleSmartDashboard and self.motorMultiplier > 1:
                SmartDashboard.putNumber("Motor Multiplier", 1)
        
        # SmartDasboard updating
        if self.toggleSmartDashboard:
            self.updateSmartDashboard()
    
    def curveOffInput(self, num, deadband=0.1): # Change deadband if necesary
        '''Does stuff like the cubing and the deadband'''
        # Do the deadband
        if num < deadband and num > -deadband:
            return 0
        # Do the cubing
        num = num ** 3
        # Do the motor multiplier (max at 1)
        if self.motorMultiplier > 1:
            self.motorMultiplier = 1
        num *= self.motorMultiplier
        return num
    
    def spinMotors(self, dir) -> None:
        '''Spins the motors (1 is intake, -1 is discharge hopefully)'''
        self.algaeMotor1.set(dir)
        self.algaeMotor2.set(-dir)
        
    def setupSmartDashboard(self) -> None:
        '''Sets up the Smart Dashboard for Algae with all the cool things'''
        SmartDashboard.putNumber("Motor1", self.algaeMotor1.get())
        SmartDashboard.putNumber("Motor2", self.algaeMotor2.get())
        SmartDashboard.putBoolean("Limit Switch", self.limitSwitch.get())
        
        SmartDashboard.putNumber("Deadband", self.deadband)
        SmartDashboard.putNumber("Motor Multiplier", self.motorMultiplier)
        SmartDashboard.putBoolean("Toggle Limit Switch", self.toggleLimitSwitch)
        
    def updateSmartDashboard(self) -> None:
        '''Updates the Smart Dashboard for Algae with all the cool things'''
        
        # Update values TO the Smart Dashboard
        SmartDashboard.putNumber("Motor1", self.algaeMotor1.get())
        SmartDashboard.putNumber("Motor2", self.algaeMotor2.get())
        SmartDashboard.putBoolean("Limit Switch", self.limitSwitch.get())
        
        # Update values FROM the Smart Dashboard
        self.deadband = SmartDashboard.getNumber("Deadband", self.deadband)
        self.motorMultiplier = SmartDashboard.getNumber("Motor Multiplier", self.motorMultiplier)
        self.toggleLimitSwitch = SmartDashboard.getBoolean("Toggle Limit Switch", self.toggleLimitSwitch)
# Thing that controls the algae

import wpilib
from wpilib import SmartDashboard
from constants import AlgaeConstants

from commands2.subsystem import Subsystem

class AlgaeHandler(Subsystem):
    '''This thing does algae intake and discharge'''
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
        dir = self.curveOffInput(controllerRightYInput, self.deadband)
        # TODO: Change this to use the one main curveOffInput thing later
        
        # Account for limit switches (if limit switch active then you can't intake)
        if self.limitSwitch.get() and dir > 0 and self.toggleLimitSwitch:
            dir = 0
        
        self.spinMotors(dir)
        
        # Change SmartDashboard motor multiplier because it can't be more than one
        if self.toggleSmartDashboard and self.motorSpeedMultiplier > 1:
                SmartDashboard.putNumber("Motor Multiplier", 1)
        if self.toggleSmartDashboard and self.motorSpeedMultiplier < -1:
                SmartDashboard.putNumber("Motor Multiplier", -1)
        
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
        if self.motorSpeedMultiplier > 1: self.motorSpeedMultiplier = 1
        if self.motorSpeedMultiplier < -1: self.motorSpeedMultiplier = -1
        
        num *= self.motorSpeedMultiplier
        
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

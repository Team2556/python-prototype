# Thing that does the coral discharge

import wpilib
from wpilib import SmartDashboard # <---- TODO
from constants import CoralConstants

from commands2.subsystem import Subsystem

class CoralDischarge(Subsystem):
    '''This thing does the coral discharge on the press (or hold) of a button.
        Uses limelight stuff to determine what side to discharge the coral'''
    def __init__(self):
        motorChannel1 = CoralConstants.kDischargeMotorID
        
        # Assuming it's PWMSparkMax motor controller for now
        # Also assuming there's only one motor
        self.coralDischargeMotor = wpilib.PWMSparkMax(motorChannel1)
        
        self.spinningDirection = 0
        
        # Temporary trigger variables
        self.leftTriggerActive = False
        self.rightTriggerActive = False
        
        # Constants to tune
        self.motorMultiplter = 1
        
    # The variables spinLeft and spinRight are temporary until auto limelight thing is added
    def spinLeft(self, isActive) -> None:
        '''Updates left trigger variable and motors when left trigger is pressed'''
        self.leftTriggerActive = isActive
        self.spinMotorsWithTwoTriggersToControl()
        
    def spinRight(self, isActive) -> None:
        '''Updates right trigger variable and motors when right trigger is pressed'''
        self.rightTriggerActive = isActive
        self.spinMotorsWithTwoTriggersToControl()
        
    def spinMotorsWithTwoTriggersToControl(self):
        '''Temporary thing until limelight automation stuff is added'''
        self.spinMotors((int(self.leftTriggerActive) * -1) + int(self.rightTriggerActive))
        
    def spinMotors(self, dir) -> None:
        '''Spins the motor in the direction (also does motor multiplier)'''
        # First update spinning direction
        self.spinningDirection = dir
        
        # Does motor multiplier
        dir *= self.motorMultiplter
        
        # Actually spins the motor(s)
        self.coralDischargeMotor.set(dir)
        
        # For testing
        print(self.spinningDirection)
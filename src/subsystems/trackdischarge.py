# Thing that does the coral discharge

import wpilib
from wpilib import SmartDashboard
from constants import CoralConstants

from commands2.subsystem import Subsystem

# How to quickly change this code from two joystick inputs to one input with limelight sensory thing
# Cool easy step-by-step tutorial quick and easy plan latest update part 2 version 5.1.0.1 ultra pro max 4k graphics premium ++ 
# The open brackets are checks by the way (unchecked: [ ]; checked: [/])
# STEP 1 [ ]: Uncomment the spin and default function
# STEP 2 [ ]: Somehow get the thing that determines whether it goes left or right
# STEP 3 [ ]: Delete spinLeft, spinRight, and spinMotorsWithTwoTriggersToControl
# STEP 4 [ ]: Change robot container to have probably the right joystick to call the spin function using whileTrue
# STEP 5 [ ]: Change SmartDashboard and delete "Left Trigger" boolean
# STEP 6 [ ]: Add spinningDirection boolean to SmartDashboard
# STEP 7 [ ]: Do the rest of the stuff that I forgot to add here

# TODO: Change file and branch to trackDischarge

class TrackDischarge(Subsystem):
    '''This thing does the coral discharge on the press (or hold) of a button.
        Uses limelight stuff to determine what side to discharge the coral'''
    def __init__(self):
        motorChannel1 = CoralConstants.kCoralMotorID
        
        # Assuming it's PWMSparkMax motor controller for now
        # Also assuming there's only one motor
        self.coralDischargeMotor = wpilib.PWMSparkMax(motorChannel1)
        
        self.spinningDirection = 0
        
        # Temporary trigger variables
        self.leftTriggerActive = False
        self.rightTriggerActive = False
        
        # Constants to tune
        self.motorMultiplier = 1
        
        # SmartDashvoard Setup
        self.setupSmartDashboard()
        
    # The variables spinLeft and spinRight are temporary until auto limelight thing is added
    def spinLeft(self, isActive) -> None:
        '''Updates left trigger variable and motors when left trigger is pressed'''
        self.leftTriggerActive = isActive
        self.spinMotorsWithTwoTriggersToControl()
        
    # Commented out until we have limelight detector data thing to steal from.
    # def spin(self, isActive):
    #     '''Spins the motor when button is pressed automatically in the right direction'''
    #     self.dischargeDirection = None # Put whatever senses the direction here preferably as a "left" or "right"
    #     if isActive:
    #         if self.dischargeDirection == "left":
    #             self.spinMotors(-1)
    #         elif self.dischargeDirection == "right":
    #             self.spinMotors(1)
    #     else:
    #         self.spinMotors(0)
    
    # def default(self):
    #     '''When the trigger isn't being pressed'''
    #     self.spinMotors(0)
        
    def spinRight(self, isActive) -> None:
        '''Updates right trigger variable and motors when right trigger is pressed'''
        self.rightTriggerActive = isActive
        self.spinMotorsWithTwoTriggersToControl()
        
    def spinMotorsWithTwoTriggersToControl(self):
        '''Temporary thing until limelight automation stuff is added'''
        self.spinMotors((int(self.leftTriggerActive) * -1) + int(self.rightTriggerActive))
        
    def spinMotors(self, dir) -> None:
        '''Spins the motor in the right direction (also does motor multiplier)'''
        # First update spinning direction
        self.spinningDirection = dir
        
        # Does motor multiplier
        if self.motorMultiplier > 1:
            self.motorMultiplier = 1
        elif self.motorMultiplier < -1:
            self.motorMultiplier = -1
        dir *= self.motorMultiplier
        
        # Actually spins the motor(s)
        self.coralDischargeMotor.set(dir)
        
        # Update SmartDashboard updating
        self.updateSmartDashboard()
        
    def setupSmartDashboard(self) -> None:
        '''Sets up the Smart Dashboard for Coral Discharge with all the cool things'''
        SmartDashboard.putNumber("Motor1", self.coralDischargeMotor.get())
        SmartDashboard.putBoolean(" - LeftTrigger", self.leftTriggerActive)
        SmartDashboard.putBoolean(" - RightTrigger", self.rightTriggerActive)
        
        SmartDashboard.putNumber("Motor Multiplier", self.motorMultiplier)
        
    def updateSmartDashboard(self) -> None:
        '''Updates the Smart Dashboard for Coral Discharge with all the cool things'''
        
        # Update values TO the Smart Dashboard
        SmartDashboard.putNumber("Motor1", self.coralDischargeMotor.get())
        SmartDashboard.putBoolean(" - LeftTrigger", self.leftTriggerActive)
        SmartDashboard.putBoolean(" - RightTrigger", self.rightTriggerActive)
        
        # Update values FROM the Smart Dashboard
        self.motorMultiplier = SmartDashboard.getNumber("Motor Multiplier", self.motorMultiplier)
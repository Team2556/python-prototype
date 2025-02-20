'''Does the algae commands'''

import wpilib

import keyboard # Figure out later

from commands2 import Command, button, cmd
from subsystems import algae

import constants

'''
Algae Control (ideal) Pseudocode:
    - If Up D-pad pressed:
        - Set pivot to intake position (90deg if 0 is straight down)
        - If limit switch active:
            - Set wheels to 0
        - Else:
            - Set wheels to intake (1)
    - If Left (or Right bc why not) D-pad pressed:
        - Set pivot to discharge position (My guess is like 25deg if 0 is straight down)
        - Set wheels to discharge (-1)
    - If Down D-pad pressed:
        - Set pivot to discharge position (0deg if 0 is straight down)
        - Set wheels to 0
'''

class AlgaeCommand(Command):
    '''Tells the algae all what to do'''
    
    def __init__(self, algaeSubsystem: algae.AlgaeHandler, joystick: button.CommandXboxController): # Currently using "joystick2" object
        
        self.algaeSubsystem = algaeSubsystem
        self.joystick = joystick
        
        self.addRequirements(self.algaeSubsystem)
        
        # CONSTANTS TO TUNE
        # Values to set pivoting motor to
        self.pivotIntakePositionValue = 0.25 # Pivot position when grabbing algae
        self.pivotProcessingValue = 0.07 # Pivot position when about to send to processor
        self.pivotIdleValue = 0 # Pivot position when idle
        # Intake wheels multiply by this speed
        self.intakeMultiplier = 0.2
        # The time it takes to switch between pivoting positions
        self.pivotTime = 1
        # The delay from spinning the wheels to spit out the algae once processing input is pressed (thats what the timer is for)
        self.processDelay = 0.5
        
        self.timer = wpilib.Timer()
        
        self.algaePivotPosition = "idle"
        
        # Testing variable that ignores limit switches if False (will always be True during games)
        self.toggleLimitSwitch = True

        # These inputs should hopefully be replaced with a keyboard thing
        self.joystick.povUp().whileTrue(
            cmd.run(lambda: self.intakePosition(), self.algaeSubsystem)
        )
        self.joystick.povRight().whileTrue(
            cmd.run(lambda: self.processingPosition(), self.algaeSubsystem)
        )
        self.joystick.povUp().whileTrue(
            cmd.run(lambda: self.idlePosition(), self.algaeSubsystem)
        )
        
    def execute(self):
        if self.timer.get() > self.pivotProcessingValue and self.algaePivotPosition == "processing":
            self.algaeSubsystem.changePosition(self.pivotProcessingValue, self.pivotTime)
            self.timer.stop()
            self.timer.reset()
    
    def intakePosition(self):
        self.algaePivotPosition = "intaking"
        self.algaeSubsystem.changePosition(self.pivotIntakePositionValue, self.pivotTime)
        if self.toggleLimitSwitch and self.algaeSubsystem.limitSwitch.get():
            self.algaeSubsystem.spinIntakeMotor(self.intakeMultiplier)
        else: 
            self.algaeSubsystem.spinIntakeMotor(0)  
    
    def processingPosition(self):
        self.algaePivotPosition = "processing"
        self.timer.restart()
        self.algaeSubsystem.changePosition(self.pivotProcessingValue, self.pivotTime)
    
    def idlePosition(self):
        self.algaePivotPosition = "idle"
        self.algaeSubsystem.changePosition(self.pivotIdleValue, self.pivotTime) 
        self.algaeSubsystem.spinIntakeMotor(0)
        
    # Do (only) tuning SmartDashboard stuff here
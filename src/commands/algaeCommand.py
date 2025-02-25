'''Does the algae commands'''

from wpilib import Timer, SmartDashboard

# import keyboard # Figure out later

from commands2 import Command, button, cmd
from subsystems import algae

from constants import AlgaeConstants

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
        
        self.timer = Timer() # (from wpilib)
        
        self.algaePivotPosition = "idle"
        
        # Testing variable that ignores limit switches if False (will always be True during games)
        self.toggleLimitSwitch = True

        # These inputs should hopefully be replaced with a keyboard thing
        self.joystick.povUp().whileTrue(
            cmd.runOnce(lambda: self.intakePosition(), self.algaeSubsystem)
        )
        self.joystick.povRight().whileTrue(
            cmd.runOnce(lambda: self.processingPosition(), self.algaeSubsystem)
        )
        self.joystick.povDown().whileTrue(
            cmd.runOnce(lambda: self.idlePosition(), self.algaeSubsystem)
        )
        
        self.setupSmartDashboard()
        
    def initialize(self):...
        # Runs when the command is scheduled, not like __init__ that rins when the class is made
        
    def execute(self):
        # Runs periodically
        # Stop intaking if limit switch active
        if self.algaePivotPosition == "intaking":
            if self.toggleLimitSwitch and self.algaeSubsystem.limitSwitch.get():
                self.algaeSubsystem.spinIntakeMotor(0)
            else: 
                self.algaeSubsystem.spinIntakeMotor(self.intakeMultiplier)
        # Start spinning the wheels the other way (self.pivotProcessingValue) after it starts processing
        if self.timer.get() > self.processDelay:
            if self.algaePivotPosition == "processing":
                self.algaeSubsystem.spinIntakeMotor(-self.intakeMultiplier)
            self.timer.stop()
            self.timer.reset()
        self.updateSmartDashboard()
    
    def intakePosition(self):
        '''Sets algae manipulator to intaking position'''
        self.algaePivotPosition = "intaking"
        self.algaeSubsystem.changePosition(self.pivotIntakePositionValue, self.pivotTime)
        print(self.algaePivotPosition)
    
    def processingPosition(self):
        '''Sets algae manipulator to processing position'''
        self.algaePivotPosition = "processing"
        self.timer.restart()
        self.algaeSubsystem.changePosition(self.pivotProcessingValue, self.pivotTime)
        print(self.algaePivotPosition)
    
    def idlePosition(self):
        '''Sets algae manipulator to idle position'''
        self.algaePivotPosition = "idle"
        self.algaeSubsystem.changePosition(self.pivotIdleValue, self.pivotTime) 
        self.algaeSubsystem.spinIntakeMotor(0)
        print(self.algaePivotPosition)
        
    # Do SmartDashboard stuff here
    
    def setupSmartDashboard(self):
        '''Sets up the NetworkTables stuff for algae info'''
        
        # Setup values TO the Dashboard
        self.updateSDInfo()
        self.setupTuningInfo()
        
    def updateSmartDashboard(self):
        '''Updates the NetworkTables stuff for algae info'''
        
        # Update values TO the Dashboard
        self.updateSDInfo()
        
        # Update values FROM the Dashboard
        self.getTuningInfo()
    
    def updateSDInfo(self):
        SmartDashboard.putString("Algae/Pivot Motor", self.algaeSubsystem.pivotMotor.get_position().__str__())
        SmartDashboard.putNumber("Algae/Intake Motor", self.algaeSubsystem.intakeMotor.get())
        SmartDashboard.putBoolean("Algae/Limit Switch", self.algaeSubsystem.limitSwitch.get())
        SmartDashboard.putNumber("Algae/Processing Timer", round(self.timer.get(), 2))
        SmartDashboard.putString("Algae/Pivot Position", self.algaePivotPosition)
    
    def setupTuningInfo(self):
        SmartDashboard.putNumber("Algae/Pivot Time", self.pivotTime)
        SmartDashboard.putNumber("Algae/Intake Multiplier", self.intakeMultiplier)
        SmartDashboard.putBoolean("Algae/Toggle Limit Switch", self.toggleLimitSwitch)
        SmartDashboard.putNumber("Algae/Processing Delay", self.processDelay)
        SmartDashboard.putNumber("Algae/Intake Position", self.pivotIntakePositionValue)
        SmartDashboard.putNumber("Algae/Processing Position", self.pivotProcessingValue)
        SmartDashboard.putNumber("Algae/Idle Position", self.pivotIdleValue)
         
    def getTuningInfo(self):
        self.pivotTime = SmartDashboard.getNumber("Algae/Pivot Time", self.pivotTime)
        self.intakeMultiplier = SmartDashboard.getNumber("Algae/Intake Multiplier", self.intakeMultiplier)
        self.toggleLimitSwitch = SmartDashboard.getBoolean("Algae/Toggle Limit Switch", self.toggleLimitSwitch)
        self.processDelay = SmartDashboard.getNumber("Algae/Processing Delay", self.processDelay)
        self.pivotIntakePositionValue = SmartDashboard.getNumber("Algae/Intake Position", self.pivotIntakePositionValue)
        self.pivotProcessingValue = SmartDashboard.getNumber("Algae/Processing Position", self.pivotProcessingValue)
        self.pivotIdleValue = SmartDashboard.getNumber("Algae/Idle Position", self.pivotIdleValue)
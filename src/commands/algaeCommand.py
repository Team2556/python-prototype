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
        
    def execute(self):
        # Runs periodically
        # Stop intaking if limit switch active
        if self.algaePivotPosition == "intaking":
            if self.toggleLimitSwitch and self.algaeSubsystem.limitSwitch.get():
                self.algaeSubsystem.spinIntakeMotor(self.intakeMultiplier)
            else: 
                self.algaeSubsystem.spinIntakeMotor(0)
        # Start spinning the wheels the other way (self.pivotProcessingValue) after it starts processing
        if self.timer.get() > self.pivotProcessingValue and self.algaePivotPosition == "processing":
            self.algaeSubsystem.changePosition(self.pivotProcessingValue, self.pivotTime)
            self.timer.stop()
            self.timer.reset()
        self.updateSmartDashboard()
        print("uw8nygtyubeigfyudgsinbydbtfgcweiugxnftuewbctfruevwtdfyuiutwey")
    
    def intakePosition(self):
        '''Sets algae manipulator to intaking position'''
        self.algaePivotPosition = "intaking"
        self.algaeSubsystem.changePosition(self.pivotIntakePositionValue, self.pivotTime)
        if self.toggleLimitSwitch and self.algaeSubsystem.limitSwitch.get():
            self.algaeSubsystem.spinIntakeMotor(self.intakeMultiplier)
        else: 
            self.algaeSubsystem.spinIntakeMotor(0)  
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
        
    # Do (only) tuning SmartDashboard stuff here
    
    def setupSmartDashboard(self):
        '''Sets up the NetworkTables stuff for algae info'''
        
        # Setup values TO the Dashboard
        SmartDashboard.putString("Algae/Pivot Motor", self.algaeSubsystem.pivotMotor.get_position().__str__())
        SmartDashboard.putNumber("Algae/Intake Motor", self.algaeSubsystem.intakeMotor.get())
        SmartDashboard.putBoolean("Algae/Limit Switch", self.algaeSubsystem.limitSwitch.get())
        
        SmartDashboard.putNumber("Algae/Pivot Time", self.pivotTime)
        SmartDashboard.putNumber("Algae/Intake Multiplier", self.intakeMultiplier)
        SmartDashboard.putBoolean("Algae/Toggle Limit Switch", self.toggleLimitSwitch)
        
    def updateSmartDashboard(self):
        '''Updates the NetworkTables stuff for algae info'''
        
        # Update values TO the Dashboard
        SmartDashboard.putString("Algae/Pivot Motor", self.algaeSubsystem.pivotMotor.get_position().__str__())
        SmartDashboard.putNumber("Algae/Intake Motor", self.algaeSubsystem.intakeMotor.get())
        SmartDashboard.putBoolean("Algae/Limit Switch", self.algaeSubsystem.limitSwitch.get())
        
        # Update values FROM the Dashboard
        self.pivotTime = SmartDashboard.getNumber("Algae/Pivot Time", self.pivotTime)
        self.intakeMultiplier = SmartDashboard.getNumber("Algae/Intake Multiplier", self.intakeMultiplier)
        self.toggleLimitSwitch = SmartDashboard.getBoolean("Algae/cvToggle Limit Switch", self.toggleLimitSwitch)

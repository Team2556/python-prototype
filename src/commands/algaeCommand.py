'''Does the algae commands'''

from wpilib import Timer, SmartDashboard

# import keyboard # Figure out later

from commands2 import Command, button, cmd
from subsystems import algae, ElevatorSubsystem

from constants import AlgaeConstants

# Hopefully temporarys
import phoenix6
import commands2

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
        
Better Cooler Updated Algae Control (more ideal) Pseudocode:
    - TODO: Do some stuff detecting if holding algae before doing any of the commands
    - If sent command to process:
        - Lower Elevator to processor height
        - Set algae handler to process position
    - Yeah the rest are self-explanatory actually
'''

class AlgaeCommand(Command):
    '''Tells the algae all what to do'''
    
    def __init__(
        self, 
        algaeSubsystem: algae.AlgaeHandler, 
        elevatorSubsystem: ElevatorSubsystem.ElevatorSubsystem,          
        joystick: button.CommandXboxController
    ): 
        # Currently using "_joystick2" object
        
        self.algaeSubsystem = algaeSubsystem
        self.elevatorSubsystem = elevatorSubsystem
        self.joystick = joystick
        
        self.addRequirements(self.algaeSubsystem, self.elevatorSubsystem)
        
        # CONSTANTS TO TUNE (maybe put in constants.py later)
        self.elevatorProcessingPositionValue = 0.01
        self.elevatorL2IntakePositionValue = 0.02
        self.elevatorL3IntakePositionValue = 0.03
        # Values to set pivoting motor to
        self.pivotIntakePositionValue = 0.25 # Pivot position when grabbing algae
        self.pivotProcessingValue = 0.07 # Pivot position when about to send to processor
        self.pivotIdleValue = 0 # Pivot position when idle
        # Intake wheels multiply by this speed
        self.intakeMultiplier = 0.2
        # The time it takes to switch between pivoting positions
        self.pivotTime = 1
        # The delay from spinning the wheels to spit out the algae once processing input is pressed (thats what the timer is for)
        self.spinProcessDelay = 0.5
        # The delay from setting the elevator to moving the pivot motor when intaking
        self.intakeDelay = 0.75
        # The delay from setting the elevator to moving the pivot motor when processing
        self.processDelay = 0.75
        
        self.timer = Timer() # (from wpilib)
        
        self.algaePivotPosition = "idle"
        
        # Testing variable that ignores limit switches if False (will always be True during games)
        self.toggleLimitSwitch = True

        # These inputs should hopefully be replaced with a keyboard thing
        self.joystick.povUp().whileTrue(
            cmd.runOnce(lambda: self.grabAlgaeFromL3(), self.algaeSubsystem)
        )
        self.joystick.povRight().whileTrue(
            cmd.runOnce(lambda: self.grabAlgaeFromL2(), self.algaeSubsystem)
        )
        self.joystick.povDown().whileTrue(
            cmd.runOnce(lambda: self.processAlgae(), self.algaeSubsystem)
        )
        
        self.setupSmartDashboard()
        
    def initialize(self):...
        # Runs when the command is scheduled, not like __init__ that rins when the class is made
        
    def execute(self):
        # Runs periodically (does all the periodic stuffs)
        # Stop intaking if limit switch active
        if self.algaePivotPosition == "intaking":
            if self.toggleLimitSwitch and self.algaeSubsystem.limitSwitch.get():
                self.algaeSubsystem.spinIntakeMotor(0)
            else: 
                self.algaeSubsystem.spinIntakeMotor(self.intakeMultiplier)
        # Start spinning the wheels the other way (self.pivotProcessingValue) after it starts processing
        if self.timer.get() > self.spinProcessDelay:
            if self.algaePivotPosition == "processing":
                self.algaeSubsystem.spinIntakeMotor(-self.intakeMultiplier)
            self.timer.stop()
            self.timer.reset()           
        self.updateSmartDashboard()
        
    
    # Functions that perform complete tasks (probably from control panel)
    
    def grabAlgaeFromL2(self):
        '''Automatically moves elevator to correct position for L2 and grabs algae'''
        if not self.algaeSubsystem.limitSwitch.get():
            self.elevatorSubsystem.update_setpoint(self.elevatorL2IntakePositionValue, incremental=False)
            self.elevatorSubsystem.moveElevator()
            # Wait a bit somehow (probably with commands2.WaitCommand)
            self.intakePosition()
        
    def grabAlgaeFromL3(self):
        '''Automatically moves elevator to correct position for L3 and grabs algae'''
        if not self.algaeSubsystem.limitSwitch.get():
            self.elevatorSubsystem.update_setpoint(self.elevatorL3IntakePositionValue, incremental=False)
            self.elevatorSubsystem.moveElevator()
            # Wait a bit somehow
            self.intakePosition()
        
    def processAlgae(self):
        '''Automatically moves elevator down to processor and process the algae'''
        if self.algaeSubsystem.limitSwitch.get():
            self.elevatorSubsystem.update_setpoint(self.elevatorProcessingPositionValue, incremental=False)
            self.elevatorSubsystem.moveElevator()
            # Wait a bit somehow
            self.processingPosition()
    
    # Functions that move the algae pivoter
    
    def intakePosition(self):
        '''Sets algae manipulator to intaking position'''
        self.algaePivotPosition = "intaking"
        self.algaeSubsystem.changePosition(self.pivotIntakePositionValue, self.pivotTime)
    
    def processingPosition(self):
        '''Sets algae manipulator to processing position'''
        self.algaePivotPosition = "processing"
        self.timer.restart()
        self.algaeSubsystem.changePosition(self.pivotProcessingValue, self.pivotTime)
    
    def idlePosition(self):
        '''Sets algae manipulator to idle position'''
        self.algaePivotPosition = "idle"
        self.algaeSubsystem.changePosition(self.pivotIdleValue, self.pivotTime) 
        self.algaeSubsystem.spinIntakeMotor(0)
        
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
        
        self.algaeSubsystem.updateSmartDashboard()
    
    def updateSDInfo(self):
        SmartDashboard.putNumber("Algae/Processing Timer", round(self.timer.get(), 2))
        SmartDashboard.putString("Algae/Pivot Instruction", self.algaePivotPosition)
    
    def setupTuningInfo(self):
        SmartDashboard.putNumber("Algae/Pivot Time", self.pivotTime)
        SmartDashboard.putNumber("Algae/Intake Multiplier", self.intakeMultiplier)
        SmartDashboard.putBoolean("Algae/Toggle Limit Switch", self.toggleLimitSwitch)
        SmartDashboard.putNumber("Algae/Processing Delay", self.spinProcessDelay)
        SmartDashboard.putNumber("Algae/Pivot Intake Position", self.pivotIntakePositionValue)
        SmartDashboard.putNumber("Algae/Pivot Processing Position", self.pivotProcessingValue)
        SmartDashboard.putNumber("Algae/Pivot Idle Position", self.pivotIdleValue)
        SmartDashboard.putNumber("Algae/Elevator Processing Position", self.elevatorProcessingPositionValue)
        SmartDashboard.putNumber("Algae/Elevator L2 Intaking Position", self.elevatorL2IntakePositionValue)
        SmartDashboard.putNumber("Algae/Elevator L3 Intaking Position", self.elevatorL3IntakePositionValue)
        SmartDashboard.putNumber("Algae/Pivot Intake Delay", self.intakeDelay)
        SmartDashboard.putNumber("Algae/Pivot Process Delay", self.processDelay)
         
    def getTuningInfo(self):
        self.pivotTime = SmartDashboard.getNumber("Algae/Pivot Time", self.pivotTime)
        self.intakeMultiplier = SmartDashboard.getNumber("Algae/Intake Multiplier", self.intakeMultiplier)
        self.toggleLimitSwitch = SmartDashboard.getBoolean("Algae/Toggle Limit Switch", self.toggleLimitSwitch)
        self.spinProcessDelay = SmartDashboard.getNumber("Algae/Processing Delay", self.spinProcessDelay)
        self.pivotIntakePositionValue = SmartDashboard.getNumber("Algae/Pivot Intake Position", self.pivotIntakePositionValue)
        self.pivotProcessingValue = SmartDashboard.getNumber("Algae/Pivot Processing Position", self.pivotProcessingValue)
        self.pivotIdleValue = SmartDashboard.getNumber("Algae/Pivot Idle Position", self.pivotIdleValue)
        self.elevatorProcessingPositionValue = SmartDashboard.getNumber("Algae/Elevator Processing Position", self.elevatorProcessingPositionValue)
        self.elevatorL2IntakePositionValue = SmartDashboard.getNumber("Algae/Elevator L2 Intaking Position", self.elevatorL2IntakePositionValue)
        self.elevatorL3IntakePositionValue = SmartDashboard.getNumber("Algae/Elevator L3 Intaking Position", self.elevatorL3IntakePositionValue)
        self.intakeDelay = SmartDashboard.putNumber("Algae/Pivot Intake Delay", self.intakeDelay)
        self.processDelay = SmartDashboard.putNumber("Algae/Pivot Process Delay", self.processDelay)
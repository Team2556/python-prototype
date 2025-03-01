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
    - If sent command to process:
        - Lower Elevator to processor height
        - Set algae handler to process position
    - Yeah the rest are self-explanatory actually so I don't have to type it
    
TODO to make this better:
    - Figure out commands2.SequentialCommandGroup 
    - Make a more efficient way to make it wait a bit to do something (maybe commands2.WaitCommand)
        
    - Get these structured in the final repository
    - Make it compatable with the new control panel
    
    - If subsystem motor code doesn't work, get that to actually work
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
        
        # Cool way to wait a bit for something to happen
        self.pivotWaitingPosition = "intaking"
        self.pivotDelayTimer = Timer()
        self.processDelayTimer = Timer() # (class from wpilib)
        
        self.algaePivotPosition = "idle"

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
        # Runs when the command is scheduled, not like __init__ that runs when the class is made
        
    def execute(self):
        # Runs periodically (does all the periodic stuffs)
        
        # Stop intaking if limit switch active
        if self.algaePivotPosition == "intaking":
            if self.algaeSubsystem.getLimitSwitchActive(AlgaeConstants.kToggleLimitSwitch):
                self.algaeSubsystem.spinIntakeMotor(0)
            else: 
                self.algaeSubsystem.spinIntakeMotor(AlgaeConstants.kIntakeMultiplier)
                
        # Start spinning the wheels the other way (self.pivotProcessingValue) after it starts processing
        if self.processDelayTimer.get() > AlgaeConstants.kSpinProcessDelayValue:
            if self.algaePivotPosition == "processing":
                self.algaeSubsystem.spinIntakeMotor(-AlgaeConstants.kIntakeMultiplier)
            self.processDelayTimer.stop()
            self.processDelayTimer.reset()
        
        # Changes pivot if self.pivotDelayTimer if higher than any of the delay values
        if self.pivotWaitingPosition == "intaking" and self.pivotDelayTimer.get() >= AlgaeConstants.kIntakeDelayValue:
            self.pivotWaitingPosition = "none"
            self.pivotDelayTimer.stop()
            self.pivotDelayTimer.reset()
            self.intakePosition()
        if self.pivotWaitingPosition == "processing" and self.pivotDelayTimer.get() >= AlgaeConstants.kProcessDelayValue:
            self.pivotWaitingPosition = "none"
            self.pivotDelayTimer.stop()
            self.pivotDelayTimer.reset()
            self.processingPosition()
        
        self.updateSmartDashboard()
        
    # Functions that perform complete tasks (probably from control panel)
    
    def grabAlgaeFromL2(self):
        '''Automatically moves elevator to correct position for L2 and grabs algae'''
        if not self.algaeSubsystem.getLimitSwitchActive(AlgaeConstants.kToggleLimitSwitch):
            self.elevatorSubsystem.update_setpoint(AlgaeConstants.kElevatorL2IntakePositionValue, incremental=False)
            self.elevatorSubsystem.moveElevator()
            # Make it set pivot to intake position after AlgaeConstants.kIntakeDelayValue seconds
            self.startPivotWaitingTimer("intaking")
        
    def grabAlgaeFromL3(self):
        '''Automatically moves elevator to correct position for L3 and grabs algae'''
        if not self.algaeSubsystem.getLimitSwitchActive(AlgaeConstants.kToggleLimitSwitch):
            self.elevatorSubsystem.update_setpoint(AlgaeConstants.kElevatorL3IntakePositionValue, incremental=False)
            self.elevatorSubsystem.moveElevator()
            # Make it set pivot to intake position after AlgaeConstants.kIntakeDelayValue seconds
            self.startPivotWaitingTimer("intaking")
        
    def processAlgae(self):
        '''Automatically moves elevator down to processor and process the algae'''
        if self.algaeSubsystem.limitSwitch.get():
            self.elevatorSubsystem.update_setpoint(AlgaeConstants.kElevatorProcessingPositionValue, incremental=False)
            self.elevatorSubsystem.moveElevator()
            # Make it set pivot to process position after AlgaeConstants.kProcessDelayValue seconds 
            self.startPivotWaitingTimer("processing")
            
    # Function that starts the timer that delays pivot moving
    
    def startPivotWaitingTimer(self, waitingPosition):
        self.pivotWaitingPosition = waitingPosition
        self.pivotDelayTimer.reset()
        self.pivotDelayTimer.start()
    
    # Functions that move the algae pivoter
    
    def intakePosition(self):
        '''Sets algae manipulator to intaking position'''
        self.algaePivotPosition = "intaking"
        self.algaeSubsystem.changePosition(AlgaeConstants.kPivotIntakePositionValue, AlgaeConstants.kPivotTime)
    
    def processingPosition(self):
        '''Sets algae manipulator to processing position'''
        self.algaePivotPosition = "processing"
        self.processDelayTimer.restart()
        self.algaeSubsystem.changePosition(AlgaeConstants.kPivotProcessingValue, AlgaeConstants.kPivotTime)
    
    def idlePosition(self):
        '''Sets algae manipulator to idle position'''
        self.algaePivotPosition = "idle"
        self.algaeSubsystem.changePosition(AlgaeConstants.kPivotIdleValue, AlgaeConstants.kPivotTime) 
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
        SmartDashboard.putNumber("Algae/Processing Delay Timer", round(self.processDelayTimer.get(), 2))
        SmartDashboard.putString("Algae/Pivot Waiting Position", self.pivotWaitingPosition)
        SmartDashboard.putNumber("Algae/Pivot Delay Timer", round(self.pivotDelayTimer.get(), 2))
        SmartDashboard.putString("Algae/Pivot Instruction", self.algaePivotPosition)
    
    def setupTuningInfo(self):
        SmartDashboard.putNumber("Algae/Pivot Time", AlgaeConstants.kPivotTime)
        SmartDashboard.putNumber("Algae/Intake Multiplier", AlgaeConstants.kIntakeMultiplier)
        SmartDashboard.putBoolean("Algae/Toggle Limit Switch", AlgaeConstants.kToggleLimitSwitch)
        SmartDashboard.putNumber("Algae/Processing Delay", AlgaeConstants.kSpinProcessDelayValue)
        SmartDashboard.putNumber("Algae/Pivot Intake Position", AlgaeConstants.kPivotIntakePositionValue)
        SmartDashboard.putNumber("Algae/Pivot Processing Position", AlgaeConstants.kPivotProcessingValue)
        SmartDashboard.putNumber("Algae/Pivot Idle Position", AlgaeConstants.kPivotIdleValue)
        SmartDashboard.putNumber("Algae/Elevator Processing Position", AlgaeConstants.kElevatorProcessingPositionValue)
        SmartDashboard.putNumber("Algae/Elevator L2 Intaking Position", AlgaeConstants.kElevatorL2IntakePositionValue)
        SmartDashboard.putNumber("Algae/Elevator L3 Intaking Position", AlgaeConstants.kElevatorL3IntakePositionValue)
        SmartDashboard.putNumber("Algae/Pivot Intake Delay", AlgaeConstants.kIntakeDelayValue)
        SmartDashboard.putNumber("Algae/Pivot Process Delay", AlgaeConstants.kProcessDelayValue)
         
    def getTuningInfo(self):
        AlgaeConstants.kPivotTime = SmartDashboard.getNumber("Algae/Pivot Time", AlgaeConstants.kPivotTime)
        AlgaeConstants.kIntakeMultiplier = SmartDashboard.getNumber("Algae/Intake Multiplier", AlgaeConstants.kIntakeMultiplier)
        AlgaeConstants.kToggleLimitSwitch = SmartDashboard.getBoolean("Algae/Toggle Limit Switch", AlgaeConstants.kToggleLimitSwitch)
        AlgaeConstants.kSpinProcessDelayValue = SmartDashboard.getNumber("Algae/Processing Delay", AlgaeConstants.kSpinProcessDelayValue)
        AlgaeConstants.kPivotIntakePositionValue = SmartDashboard.getNumber("Algae/Pivot Intake Position", AlgaeConstants.kPivotIntakePositionValue)
        AlgaeConstants.kPivotProcessingValue = SmartDashboard.getNumber("Algae/Pivot Processing Position", AlgaeConstants.kPivotProcessingValue)
        AlgaeConstants.kPivotIdleValue = SmartDashboard.getNumber("Algae/Pivot Idle Position", AlgaeConstants.kPivotIdleValue)
        AlgaeConstants.kElevatorProcessingPositionValue = SmartDashboard.getNumber("Algae/Elevator Processing Position", AlgaeConstants.kElevatorProcessingPositionValue)
        AlgaeConstants.kElevatorL2IntakePositionValue = SmartDashboard.getNumber("Algae/Elevator L2 Intaking Position", AlgaeConstants.kElevatorL2IntakePositionValue)
        AlgaeConstants.kElevatorL3IntakePositionValue = SmartDashboard.getNumber("Algae/Elevator L3 Intaking Position", AlgaeConstants.kElevatorL3IntakePositionValue)
        AlgaeConstants.kIntakeDelayValue = SmartDashboard.putNumber("Algae/Pivot Intake Delay", AlgaeConstants.kIntakeDelayValue)
        AlgaeConstants.kProcessDelayValue = SmartDashboard.putNumber("Algae/Pivot Process Delay", AlgaeConstants.kProcessDelayValue)
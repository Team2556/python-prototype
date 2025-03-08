'''Does the algae commands'''

from wpilib import Timer, SmartDashboard

# import keyboard # Figure out later

from commands2 import Command, button, cmd
import wpilib.event
from subsystems import ElevatorSubsystem, algaeSubsystem

from constants import AlgaeConstants

# Hopefully temporarys
import phoenix6
import commands2
import wpilib

'''
Control inputs to commands pseudocode:
TODO: For the commands that interact with the reef, maybe somehow center with the apriltag first
    - if L2: SequentialCommand(
        moveElevatorToL2, 
        (algaeIntakePosition and spinMotorsToIntake), 
        wait(timeItTakesToIntake), 
        stopSpinMotors, 
        algaeIdlePosition
    )
    - if L3: SequentialCommand(
        moveElevatorToL3, 
        (algaeIntakePosition and spinMotorsToIntake), 
        wait(timeItTakesToIntake), 
        stopSpinMotors, 
        algaeIdlePosition
    )
    - if Ground Intake: SequentialCommand(
        moveElevatorToGroundPosition, 
        (algaeIntakePosition and spinMotorsToIntake), 
        wait(timeItTakesToIntake), 
        stopSpinMotors, 
        algaeIdlePosition
    )
    - if Processing: SequentialCommand(
        moveElevatorToProcessing, 
        algaeProcessingPosition, 
        spinMotorsToProcess, 
        wait(timeItTakesToProcess), 
        stopSpinMotors, 
        algaeIdlePosition
    )
    - if idle: SequentialCommand(
        stopSpinMotors, 
        algaeIdlePosition
    )
    
So Commands Needed:
    - Move elevator to (all the values)
    - Move Pivot to (all the values)
    - Change Intake Motor Speed (using runOnce)
    - Parallel and Sequential Comamnds and WaitCommands
''' 

# commands2.SequentialCommandGroup

class AlgaeCommand(Command):
    '''Parent class for all the more specified algae commands'''
    
    def __init__(
        self, 
        algaeSubsystem: algaeSubsystem.AlgaeSubsystem, 
        elevatorSubsystem: ElevatorSubsystem.ElevatorSubsystem,
    ): 
        # NOT Currently using "_joystick2" object because new commands system
        
        self.algaeSubsystem = algaeSubsystem
        self.elevatorSubsystem = elevatorSubsystem
        # self.joystick = joystick
        
        # Hopefully makes it so if another command is trying to move elevator or algae, STOP IT HAHAHHAHAHA
        self.addRequirements(self.algaeSubsystem, self.elevatorSubsystem)
        self.InterruptionBehavior = commands2.InterruptionBehavior.kCancelIncoming
        
        # Cool way to wait a bit for something to happen
        self.pivotWaitingPosition = "intaking"
        self.pivotDelayTimer = Timer()
        self.processDelayTimer = Timer() # (class from wpilib)
        
        self.algaePivotPosition = "idle"

        # These inputs should hopefully be replaced with a keyboard thing
        # TODO: Replace this with the new commands system
        # self.joystick.povUp().whileTrue(
        #     cmd.runOnce(lambda: self.grabAlgaeFromL3(), self.algaeSubsystem)
        # )
        # self.joystick.povRight().whileTrue(
        #     cmd.runOnce(lambda: self.grabAlgaeFromL2(), self.algaeSubsystem)
        # )
        # self.joystick.povDown().whileTrue(
        #     cmd.runOnce(lambda: self.processAlgae(), self.algaeSubsystem)
        # )
        
        commands2.SequentialCommandGroup()
        
    def initialize(self):...
        # Runs when the command is scheduled, not like __init__ that runs when the class is made
    
    def execute(self):
        # Runs periodically (does all the periodic stuffs)
        self.updateSmartDashboard()
        return "So the rest of the function doesn't run"
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
        self.algaeSubsystem.changePosition(AlgaeConstants.kPivotReefIntakingValue)
    
    def processingPosition(self):
        '''Sets algae manipulator to processing position'''
        self.algaePivotPosition = "processing"
        self.processDelayTimer.restart()
        self.algaeSubsystem.changePosition(AlgaeConstants.kPivotProcessingValue)
    
    def idlePosition(self):
        '''Sets algae manipulator to idle position'''
        self.algaePivotPosition = "idle"
        self.algaeSubsystem.changePosition(AlgaeConstants.kPivotIdleValue)
        self.algaeSubsystem.spinIntakeMotor(0)
    
class AlgaePivotCommand(AlgaeCommand):
    '''Sets algae pivot to reef intake position'''
    def __init__(self, algaeSubsystem: algaeSubsystem.AlgaeSubsystem, elevatorSubsystem: ElevatorSubsystem.ElevatorSubsystem):
        self.algaeSubsystem = algaeSubsystem
        self.elevatorSubsystem = elevatorSubsystem
        self.addRequirements(self.algaeSubsystem, self.elevatorSubsystem)
    
    def initialize(self):
        self.algaeSubsystem.updatePivotSetpoint((AlgaeConstants.kPivotReefIntakingValue))
        self.algaeSubsystem.changePivotPosition()
        self.algaeSubsystem.spinIntakeMotor(1 * AlgaeConstants.kIntakeMultiplier)
        
    def isFinished(self):
        return self.isAccurateEnough(
            value=self.algaeSubsystem.pivotMotor.get_position(),
            targetedValue=AlgaeConstants.kPivotGroundIntakingValue
        )
        
    def end(self): pass

# # FOR TESTING COMMANDS2
# z = commands2.Command()
# x = commands2.CommandScheduler()
# y = wpilib.event.EventLoop()
# y.bind()
# x.setActiveButtonLoop()
# x.run()
# x.registerSubsystem()
# commands2.cmd.sequence()
# commands2.cmd.sequence()
# commands2.command





















'''
NOTE: All this here is outdated but I put it on the bottom in case I need to for some reason
...
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
    
What it ACTUALLY needs to do (when intaking) (in order):
    - Change Elevator to specific level
    - Change pivot to intake position
    - Spin intake motors
    - Now move pivot thingy back
    - Then move elevator DOWN

TODO to make this better:
    - Hard Stuff:
        - What's capping motor speed at 0.25 maybe
        - Make a more efficient way to make it wait a bit to do something (maybe commands2.WaitCommand)
        - If subsystem motor code doesn't work, get that to actually work
        - Figure out commands2.SequentialCommandGroup    
    - Get these structured in the final repository
    - Make it compatable with the new control panel
''' 
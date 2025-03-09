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
    
class AlgaePivotCommand(Command):
    '''Sets algae pivot motor to any position in rotations'''
    def __init__(self, position, algaeSubsystem: algaeSubsystem.AlgaeSubsystem):
        self.algaeSubsystem = algaeSubsystem
        self.addRequirements(self.algaeSubsystem)
        self.InterruptionBehavior = commands2.InterruptionBehavior.kCancelIncoming
        self.position = position
    
    def initialize(self):
        self.algaeSubsystem.updatePivotSetpoint(self.position)
        self.algaeSubsystem.changePivotPosition()
        
    def isFinished(self):
        value = self.algaeSubsystem.pivotMotor.get_position()
        if self.position == AlgaeConstants.kPivotIdleValue:
            return True # Automatically finish the command if it's being told to set to idle value
        else:
            return (value <= self.position + AlgaeConstants.kTargetValueAccuracy
                and value >= self.position - AlgaeConstants.kTargetValueAccuracy)
        
    def end(self): pass

class AlgaeIntakeCommand(Command):
    '''Super simple command that sets intake motors and that's it'''
    def __init__(self, speed, algaeSubsystem: algaeSubsystem.AlgaeSubsystem):
        self.algaeSubsystem = algaeSubsystem
        # Doesn't add requirements so it can be run at the same time as tha AlgaePivotCommand
        self.speed = speed
        
    def initialize(self):
        self.algaeSubsystem.spinIntakeMotor(self.speed)
        
    def isFinished(self): return True
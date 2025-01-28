import wpilib
from constants import AlgaeConstants

from commands2.subsystem import Subsystem
from commands2.command import Command
import commands2.cmd

# TODO: Figure out the limit switch thing
# TODO: Add necesary stuffs to physics.py (later)

class AlgaeHandler(Subsystem):
    '''This thing uses two inputs to intake or discharge'''
    def __init__(self):
        # We don't know the motor channel yet
        motorChannel1 = AlgaeConstants.kIntakeCANAddress1
        motorChannel2 = AlgaeConstants.kIntakeCANAddress2
        
        # 1 = The one spinning clockwise when intaking
        # 2 = The one spinning counterclockwise when intaking
        self.AlgaeMotor1 = wpilib.PWMSparkMax(motorChannel1)
        self.AlgaeMotor2 = wpilib.PWMSparkMax(motorChannel2)
        # set() function makes these turn
        
        # Records direction of the motors (1 is intake, -1 is discharge)
        self.spinningDirection = 0
        
        # Records when button is pressing so override system works
        self.intakeActive = False
        self.dischargeActive = False
        
        # Most of the complicated stuff is from the override system
        # where if you press one button while another is already pressed, it registeres the new button
        
    def default(self) -> None:
        '''This command runs when no others are running in the same tick'''
        # Nothing here yet but it might be needed later
        pass
    
    def intakeButtonPressed(self) -> None:
        '''Starts spinning motors in intake direction once intake button is pressed'''
        self.intakeActive = True
        self.spinMotors(1)
        
    def intakeButtonReleased(self) -> None:
        '''Spins a different motor when intake button is released; either discharge or stop the motors'''
        self.intakeActive = False
        self.spinBasedOnButtons()
        
    def dischargeButtonPressed(self) -> None:
        '''Starts spinning motors in discharge direction once discharge button is pressed'''
        self.dischargeActive = True
        self.spinMotors(-1)
        
    def dischargeButtonReleased(self) -> None:
        '''Spins a different motor when discharge button is released; either intake or stop the motors'''
        self.dischargeActive = False
        self.spinBasedOnButtons()
            
    def spinBasedOnButtons(self) -> None:
        '''Activated when a button is released because then 0 or 1 algae buttons will be held, 
            and whatever button is activated can be used to spin the motors in the right direction'''
        if self.intakeActive:
            self.spinMotors(1)
        elif self.dischargeActive:
            self.spinMotors(-1)
        else:
            self.spinMotors(0)
    
    def spinMotors(self, dir) -> None:
        '''Spins the motors (1 is intake, -1 is discharge hopefully)'''
        # Update spinningDirection variable
        self.spinningDirection = dir
        # Spins them in seperate directions to suck in the algae
        self.AlgaeMotor1.set(dir)
        self.AlgaeMotor2.set(-dir)
        # For testing
        print(self.spinningDirection)
        
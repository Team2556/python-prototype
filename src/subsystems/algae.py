import wpilib
from constants import AlgaeConstants

from commands2.subsystem import Subsystem
from commands2.command import Command
import commands2.cmd

class AlgaeHandler(Subsystem):
    
    def __init__(self):
        # We don't know the motor channel yet
        motorChannel1 = AlgaeConstants.kIntakeCANAddress1
        motorChannel2 = AlgaeConstants.kIntakeCANAddress2
        
        # 1 = The one spinning clockwise when intaking
        # 2 = The one spinning counterclockwise when intaking
        self.AlgaeMotor1 = wpilib.PWMSparkMax(motorChannel1)
        self.AlgaeMotor2 = wpilib.PWMSparkMax(motorChannel2)
        # set() function makes these turn
        
        self.holdingAlgae = False
        
        # Setup the timer
        self.timer = wpilib.Timer()
        
    def disable(self):
        self.AlgaeMotor1.set(0)
        self.AlgaeMotor2.set(0)
        print("DEFAULT COMMAND")
        
    def apply_request(self, function):
        return commands2.cmd.run(function, self)
    
    def cycle(self):
        '''Activates when algae button is pressed'''
        # Figure out some way to detect if holding algae (limit switch or something)
        # For now it just alternates every time the button is pressed but thats not the best idea
        
        # Reset timer to 0 and start
        self.timer.reset()
        self.timer.start()
        
        if self.holdingAlgae: 
            self.spinMotors(-1) # Opposite of intake
        else: 
            self.spinMotors(1) # Intake
        
        # Later replace this with some code at the start of the function that
        # sets self.holdingAlgae based on if the robot is actually holding algae
        self.holdingAlgae = not self.holdingAlgae
        
        print("BUTTON PRESSED")
                
    def checkCycle(self):
        '''If motors have been spinning for a set amount of time (AlgaeConstants.intakeTime), stop the motors'''
        if self.timer.hasElapsed(AlgaeConstants.intakeTime):
            # Reset timer to 0 and start
            self.timer.stop()
            self.timer.reset()
            self.spinMotors(0) # Stops motors
            
    def spinMotors(self, dir):
        '''Spins the motors (1 is intake, -1 spits the algae out hopefully)'''
        self.AlgaeMotor1.set(dir)
        self.AlgaeMotor2.set(-dir)
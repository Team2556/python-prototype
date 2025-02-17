# Thing that controls the algae

import wpilib
from wpilib import SmartDashboard
from constants import AlgaeConstants

from commands2.subsystem import Subsystem

import phoenix6

'''
What this does (so far hopefully I think):
- One button does a "pivoting thing" (pushes the algae into the storage?)
- One joystick does the spinny wheels (but maybe not)

- For the pivoting, there will be three values to set the angle to:
    - Idle, intaking, and processing
- Three different buttons should set to all of these

- Extra Idea: Automatically move the wheels depending on the set position
    - Probably needs some delay (which will be a costant) to let motors pivot correctly
    - It will have to check elevator position for intaking and processing (so robotstate file yay)
    
Algae Control (ideal) Pseudocode:
    - If Up D-pad pressed:
        - Set pivot to intake position (90deg if 0 is straight down)
        - If limit switch active:
            - Set wheels to 0
        - Else:
            - Set wheels to intake (1)
    - Elif Left (or Right bc why not) D-pad pressed:
        - Set pivot to discharge position (My guess is like 25deg if 0 is straight down)
        - Set wheels to discharge (-1)
    - Elif Down D-pad pressed:
        - Set pivot to discharge position (0deg if 0 is straight down)
        - Set wheels to 0
'''
# TODO: The pseudocode above is not started yet so umm... (do it)

class AlgaeHandler(Subsystem):
    '''This thing does algae intake and discharge'''
    def __init__(self):
        # Declare motor controllers
        self.pivotMotor = phoenix6.hardware.TalonFX(AlgaeConstants.kPivotMotorChannel, "rio")
        self.intakeMotor = phoenix6.hardware.TalonFX(AlgaeConstants.kIntakeWheelsChannel, "rio")
        # Declare limit switch
        self.limitSwitch = wpilib.DigitalInput(AlgaeConstants.kAlgaeLimitSwitchChannel)
        
        # Make it so if motor is set at 0 then it resists any turning if force is applied to it
        # So algae doesn't fall out
        self.pivotMotor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.intakeMotor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        
        # Creates a "CTRE Control Request Object"
        self.velocityOut = phoenix6.controls.VoltageOut(output=0)
                
        # Settings to tune for intake multiplier
        self.deadband = 0.1
        self.intakeMultiplier = 1 # Needs to be between -1 and 1
        
        # Testing variable that ignores limit switches if False (will always be True during games)
        self.toggleLimitSwitch = True
        
        # SmartDasboard setting up
        self.toggleSmartDashboard = True
        if self.toggleSmartDashboard:
            self.setupSmartDashboard()
    
    def cycle(self, controllerRightYInput) -> None:
        '''Gets called periodically; updates the algae motors based on the controller'''
        
        # Change the input stuffs
        speed = self.curveOffInput(controllerRightYInput, self.deadband)
        # TODO: Change this to use the one main curveOffInput thing later (actually not really)
        
        # Account for limit switches (if limit switch active then you can't intake)
        if self.limitSwitch.get() and speed > 0 and self.toggleLimitSwitch:
            speed = 0
        
        self.spinMotors(speed)
        
        # Change SmartDashboard motor multiplier because it can't be more than one
        if self.toggleSmartDashboard and self.motorSpeedMultiplier > 1:
                SmartDashboard.putNumber("Motor Multiplier", 1)
        if self.toggleSmartDashboard and self.motorSpeedMultiplier < -1:
                SmartDashboard.putNumber("Motor Multiplier", -1)
        
        # SmartDasboard updating
        if self.toggleSmartDashboard:
            self.updateSmartDashboard()
    
    def curveOffInput(self, speed, deadband=0.1):
        '''Does stuff like the cubing and the deadband'''
        # Do the deadband
        if speed < deadband and speed > -deadband:
            return 0
        
        # Do the cubing
        speed = speed ** 3
        
        # Do the motor multiplier (from 1 to -1)
        if self.intakeMultiplier > 1: self.intakeMultiplier = 1
        if self.intakeMultiplier < -1: self.intakeMultiplier = -1
        
        speed *= self.intakeMultiplier
        
        return speed
    
    def spinMotors(self, speed) -> None:
        '''Spins the motors (speed=1 is intake, speed=-1 is discharge hopefully)'''
        self.intakeMotor.set_control(self.velocityOut.with_output(speed))
        
    def setupSmartDashboard(self) -> None:
        '''Sets up the Smart Dashboard for Algae with all the cool things'''
        SmartDashboard.putString("Pivot Motor", self.pivotMotor.get_position().__str__())
        SmartDashboard.putNumber("Intake Motor", self.intakeMotor.get())
        SmartDashboard.putBoolean("Limit Switch", self.limitSwitch.get())
        
        SmartDashboard.putNumber("Intake Deadband", self.deadband)
        SmartDashboard.putNumber("Intake Multiplier", self.intakeMultiplier)
        SmartDashboard.putBoolean("Toggle Limit Switch", self.toggleLimitSwitch)
        
    def updateSmartDashboard(self) -> None:
        '''Updates the Smart Dashboard for Algae with all the cool things'''
        
        # Update values TO the Smart Dashboard
        SmartDashboard.putString("Pivot Motor", self.pivotMotor.get_position().__str__())
        SmartDashboard.putNumber("Intake Motor", self.intakeMotor.get())
        SmartDashboard.putBoolean("Limit Switch", self.limitSwitch.get())
        
        # Update values FROM the Smart Dashboard
        self.deadband = SmartDashboard.getNumber("Intake Deadband", self.deadband)
        self.motorSpeedMultiplier = SmartDashboard.getNumber("Intake Multiplier", self.intakeMultiplier)
        self.toggleLimitSwitch = SmartDashboard.getBoolean("Toggle Limit Switch", self.toggleLimitSwitch)

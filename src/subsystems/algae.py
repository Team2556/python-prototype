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
    - If Left (or Right bc why not) D-pad pressed:
        - Set pivot to discharge position (My guess is like 25deg if 0 is straight down)
        - Set wheels to discharge (-1)
    - If Down D-pad pressed:
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
        
        self.intakeMultiplier = AlgaeConstants.kIntakeMultiplier # Needs to be between -1 and 1
        self.pivotTime = AlgaeConstants.kPivotTime
        
        # Testing variable that ignores limit switches if False (will always be True during games)
        self.toggleLimitSwitch = True
        
        # SmartDasboard setting up
        self.toggleSmartDashboard = True
        if self.toggleSmartDashboard:
            self.setupSmartDashboard()
            
    # These are the three set stating functions
    def idle(self):
        self.changePosition(AlgaeConstants.kPivotIntakePositionValue, AlgaeConstants.kPivotTime)
        self.spinIntakeMotor(0)
        self.updateSmartDashboard()
        
    def processing(self):
        self.changePosition(AlgaeConstants.kPivotProcessingValue, AlgaeConstants.kPivotTime)
        self.spinIntakeMotor(-self.intakeMultiplier)
        self.updateSmartDashboard()
    
    def intaking(self):
        self.changePosition(AlgaeConstants.kPivotIdleValue, AlgaeConstants.kPivotTime)
        if self.toggleLimitSwitch and self.limitSwitch.get():
            self.spinIntakeMotor(self.intakeMultiplier)
        elif self.toggleLimitSwitch and not self.limitSwitch.get():
            self.spinIntakeMotor(0)
        # Cool trick: when simulating, self.limitSwitch.get() is always true
        # so you can use self.toggleLimitSwitch() to test if this works when simulating
        self.updateSmartDashboard()
    
    def changePosition(self, position, seconds):
        '''Sets the position of the pivot motor (obvoiusly)'''
        # Seconds is the time it takes to move to that position because PID is boring
        self.intakeMotor.set_position(position, seconds)
    
    def spinIntakeMotor(self, speed) -> None:
        '''Spins the intake motor (speed=1 is intake, speed=-1 is discharge hopefully)'''
        # self.intakeMotor.set_control(self.velocityOut.with_output(speed))
        self.intakeMotor.set(speed)
        
    def setupSmartDashboard(self) -> None:
        '''Sets up the Smart Dashboard for Algae with all the cool things'''
        SmartDashboard.putString("Algae/Pivot Motor", self.pivotMotor.get_position().__str__())
        SmartDashboard.putNumber("Algae/Intake Motor", self.intakeMotor.get())
        SmartDashboard.putBoolean("Algae/Limit Switch", self.limitSwitch.get())
        
        SmartDashboard.putNumber("Algae/Pivot Time", self.pivotTime)
        SmartDashboard.putNumber("Algae/Intake Multiplier", self.intakeMultiplier)
        SmartDashboard.putBoolean("Algae/Toggle Limit Switch", self.toggleLimitSwitch)
        
    def updateSmartDashboard(self) -> None:
        '''Updates the Smart Dashboard for Algae with all the cool things'''
        
        # Update values TO the Smart Dashboard
        print(self.pivotMotor.get_position().__str__())
        SmartDashboard.putString("Algae/Pivot Motor", self.pivotMotor.get_position().__str__())
        SmartDashboard.putNumber("Algae/Intake Motor", self.intakeMotor.get())
        SmartDashboard.putBoolean("Algae/Limit Switch", self.limitSwitch.get())
        
        # Update values FROM the Smart Dashboard
        self.pivotTime = SmartDashboard.getNumber("Algae/Pivot Time", self.pivotTime)
        self.motorSpeedMultiplier = SmartDashboard.getNumber("Algae/Intake Multiplier", self.intakeMultiplier)
        self.toggleLimitSwitch = SmartDashboard.getBoolean("Algae/cvToggle Limit Switch", self.toggleLimitSwitch)

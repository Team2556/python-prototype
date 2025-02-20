# Thing that controls the algae

import wpilib
from wpilib import SmartDashboard
from constants import AlgaeConstants

from commands2.subsystem import Subsystem

import phoenix6

class AlgaeHandler(Subsystem):
    '''This thing does algae intake and discharge. '''
    
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
    
    # Motor spinning functions
    def changePosition(self, position, seconds):
        '''Sets the position of the pivot motor (obvoiusly)'''
        # Seconds is the time it takes to move to that position because PID is boring
        self.pivotMotor.set_position(position, seconds)
    
    def spinIntakeMotor(self, speed) -> None:
        '''Spins the intake motor (speed=1 is intake, speed=-1 is discharge hopefully)'''
        # self.intakeMotor.set_control(self.velocityOut.with_output(speed))
        self.intakeMotor.setVoltage(speed)
        print(self.intakeMotor.get_motor_voltage())
        
    # Do only data getting SmartDashboard stuff here
        
    # The SmartDashboard/NetworkTables stuffs
    # def setupSmartDashboard(self) -> None:
    #     '''Sets up the Smart Dashboard for Algae with all the cool things'''
        
    #     SmartDashboard.putString("Algae/Pivot Motor", self.pivotMotor.get_position().__str__())
    #     SmartDashboard.putNumber("Algae/Intake Motor", self.intakeMotor.get())
    #     SmartDashboard.putBoolean("Algae/Limit Switch", self.limitSwitch.get())
        
    #     SmartDashboard.putNumber("Algae/Pivot Time", self.pivotTime)
    #     SmartDashboard.putNumber("Algae/Intake Multiplier", self.intakeMultiplier)
    #     SmartDashboard.putBoolean("Algae/Toggle Limit Switch", self.toggleLimitSwitch)
        
    # def updateSmartDashboard(self) -> None:
    #     '''Updates the Smart Dashboard for Algae with all the cool things'''
        
    #     # Update values TO the Smart Dashboard
    #     SmartDashboard.putString("Algae/Pivot Motor", self.pivotMotor.get_position().__str__())
    #     SmartDashboard.putNumber("Algae/Intake Motor", self.intakeMotor.get())
    #     SmartDashboard.putBoolean("Algae/Limit Switch", self.limitSwitch.get())
        
    #     # Update values FROM the Smart Dashboard
    #     self.pivotTime = SmartDashboard.getNumber("Algae/Pivot Time", self.pivotTime)
    #     self.intakeMultiplier = SmartDashboard.getNumber("Algae/Intake Multiplier", self.intakeMultiplier)
    #     self.toggleLimitSwitch = SmartDashboard.getBoolean("Algae/cvToggle Limit Switch", self.toggleLimitSwitch)

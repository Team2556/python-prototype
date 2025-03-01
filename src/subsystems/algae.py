# Thing that controls the algae

from wpilib import SmartDashboard, DigitalInput
from constants import AlgaeConstants

from commands2.subsystem import Subsystem

import phoenix6

class AlgaeHandler(Subsystem):
    '''This thing does algae intake and discharge.'''
    
    def __init__(self):
        # Declare motor controllers
        self.pivotMotor = phoenix6.hardware.TalonFX(AlgaeConstants.kPivotMotorChannel, "rio")
        self.intakeMotor = phoenix6.hardware.TalonFX(AlgaeConstants.kIntakeWheelsChannel, "rio")
        # Declare limit switch
        self.limitSwitch = DigitalInput(AlgaeConstants.kAlgaeLimitSwitchChannel)
        
        # Make it so if motor is set at 0 then it resists any turning if force is applied to it
        # So algae doesn't fall out
        self.pivotMotor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.intakeMotor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        
        # Creates a "CTRE Control Request Object"
        self.velocityOut = phoenix6.controls.VoltageOut(output=0)
        # self.intakeMotor.get_acceleration
        
    # Motor spinning functions
    def changePosition(self, position, seconds):
        '''Sets the position of the pivot motor (obvoiusly)'''
        # Seconds is the time it takes to move to that position because PID is boring
        self.pivotMotor.set_position(position, seconds)
    
    def spinIntakeMotor(self, speed) -> None:
        '''Spins the intake motor (speed=1 is intake, speed=-1 is discharge hopefully)'''
        # volts = 12
        # self.intakeMotor.set_control(self.velocityOut.with_output(speed * volts))
        # self.intakeMotor.setVoltage(speed * 12)
        self.intakeMotor.set(speed)
        
    def getLimitSwitchActive(self, toggleLimitSwitch: bool) -> bool:
        '''Returns true if limit switch is active (and parameter is also true)'''
        return self.limitSwitch.get() and toggleLimitSwitch
        
    def updateSmartDashboard(self):
        SmartDashboard.putString("Algae/Pivot Position", self.pivotMotor.get_position().__str__())
        SmartDashboard.putNumber("Algae/Intake Speed", self.intakeMotor.get())
        SmartDashboard.putBoolean("Algae/Limit Switch", self.limitSwitch.get())
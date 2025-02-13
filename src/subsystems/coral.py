# Theoretically, this file should automatically control the intake and discharge of coral using
# all the limelight and vision stuff. (Except for some emergency controls)

'''
Coral Control Pseudocode:
    timerValue = 1.5
    - If Discharge Button Pressed:
        - Set Motors to the correct Discharge direction
        - Reset Timer1 to 0
    - Else if Timer1 > timerValue:
        - If Coral detected on track (via breaker lights):
            - Set Motors to the correct Intake direction (based on the breaker lights)
        - Else:
            - Set Motors to 0
'''

import wpilib
from wpilib import DigitalInput, SmartDashboard
from constants import CoralConstants

from commands2.subsystem import Subsystem

import rev

# TODO: Figure out how to delete the log (hoot) stuff better

class CoralTrack(Subsystem):
    '''
        Handles all the coral track stuff; the centering and discharge.
        Uses one controller input (press to discharge)
    '''
    def __init__(self):
        # These should be tuned sometime later
        self.centerMultiplier = 0.08
        self.dischargeMultiplier = 0.16
        self.coralDischargeTime = 1.5
        
        # I'm assuming we're using rev.SparkFlex documantation
        self.motorController = rev.SparkFlex(CoralConstants.kCoralMotorPort, rev.SparkFlex.MotorType.kBrushless)
        self.globalConfig = rev.SparkMaxConfig()
        self.motorControllerConfig = rev.SparkMaxConfig()
        self.motorControllerConfig.apply(self.globalConfig)
        self.motorController.configure(
            self.globalConfig,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )
        # Put breaker light stuff here
        self.leftBreakerLight = DigitalInput(0)
        self.rightBreakerLight = DigitalInput(1)
        
        # Setup the Timer
        self.timer = wpilib.Timer()
        self.timer.start()
        
        # Set up the SmartDashboard variables
        self.setupSmartDashboard()
        
    def center(self):
        '''Centers the coral on the track using two breaker lights'''
        # The rest of the stuff here is Aidan's problem (hopefully)
        
        ''' Will center the coral by making sure both robot beambrakes see a placed coral '''

        if self.rightBreakerLight.get() and not self.leftBreakerLight.get(): # if right beambrake is sensing something but not the left beambrake, then:
            # Move track left
            self.spinMotors(-self.centerMultiplier)
        elif self.leftBreakerLight.get() and not self.rightBreakerLight.get(): # elif left beambrake is sensing something but not the right beambrake, then:
            # Move track right
            self.spinMotors(self.centerMultiplier)
        elif self.leftBreakerLight.get() and self.rightBreakerLight.get(): # elif left beambrake AND right beambrake senses something, then:
            # Stop moving track
            self.spinMotors(0)
        self.updateSmartDashboard() # Keep this at the end of the method
        
    def discharge(self):
        '''Should be doing the discharge when a button is pressed
            "direction" is 1 or -1; indicating the direction the track discharges the coral'''
            
        direction = 1 # Temporary until it can automatically get the direction
        # direction = self.getCorrectDirectionToDischargeCoral()
            
        self.timer.reset()
        self.spinMotors(direction * self.dischargeMultiplier)
        
        self.updateSmartDashboard() # Keep this at the end of the method
        
    def default(self):
        '''When discharge button is not pressed'''
        if self.timer.get() > self.coralDischargeTime:
            beamBrakerLightsAddedToRobot = False
            if beamBrakerLightsAddedToRobot:
                self.center()
                
    def spinMotors(self, value):
        '''Function that handles all the motors '''
        self.motorController.set(value)
        
    def setupSmartDashboard(self) -> None:
        '''Sets up the Smart Dashboard for with all the cool things'''
        SmartDashboard.putNumber("Coral Track Motor", self.motorController.get())
        
        SmartDashboard.putNumber("Center Multiplier", self.centerMultiplier)
        SmartDashboard.putNumber("Discharge Multiplier", self.dischargeMultiplier)
        SmartDashboard.putNumber("Discharge Time", self.coralDischargeTime)
        SmartDashboard.putBoolean(" - Left Breaker Light", self.leftBreakerLight)
        SmartDashboard.putBoolean(" - Right Breaker Light", self.rightBreakerLight)
        # Put more stuff here maybe
        
    def updateSmartDashboard(self) -> None:
        '''Updates the Smart Dashboard for with all the cool things'''
        
        # Update values TO the Smart Dashboard (put stuff here)
        SmartDashboard.putNumber("Coral Track Motor", self.motorController.get())
        SmartDashboard.putBoolean(" - Left Breaker Light", self.leftBreakerLight)
        SmartDashboard.putBoolean(" - Right Breaker Light", self.rightBreakerLight)
        
        # Update values FROM the Smart Dashboard
        self.centerMultiplier = SmartDashboard.getNumber("Center Multiplier", self.centerMultiplier)
        self.dischargeMultiplier = SmartDashboard.getNumber("Discharge Multiplier", self.dischargeMultiplier)
        self.coralDischargeTime = SmartDashboard.getNumber("Discharge Time", self.coralDischargeTime)
        
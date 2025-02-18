# Theoretically, this file should automatically control the intake and discharge of coral using
# all the limelight and vision stuff. (Except for some emergency controls)

'''
NEW Coral Control Pseudocode (NOT currently implemented in theory):
    timerValue = 1.5
    - If Discharge Button Pressed:
        - Discharge()
        - Reset Timer1 to 0
    - Else if Timer1 > timerValue:
        - Intake()
    - Else:
        - Discharge()
        
    def Discharge():
        - Spin coral track in correct direction
        - If evevatorSetPoint == L4 and no breaker lights detecting coral:
            - Activate flippers
        - Else:
            - Deactivate flippers
        
    def Intake():
        - Move motors in correct direction based on breaker lights
'''

import wpilib
from wpilib import DigitalInput, SmartDashboard
from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets
import ntcore._ntcore

from constants import CoralConstants

from commands2.subsystem import Subsystem

import rev

class CoralTrack(Subsystem):
    '''
        Handles all the coral track stuff; the centering and discharge.
        Uses one controller input (press to discharge)
    '''
    
    # NOTE: "left" and "right" are relative to behind the robot (if the elevator is on the front side)
    
    def __init__(self):
        # TODO: Figure out is setting motors to 1 means right
        
        # These should be tuned sometime later
        self.trackCenterMultiplier = CoralConstants.kTrackCenterMultiplier
        self.trackDischargeMultiplier = CoralConstants.kTrackDischargeMultiplier
        self.coralDischargeTime = CoralConstants.kCoralDischargeTime
        
        # I'm assuming we're using rev.SparkFlex documantation
        self.trackMotor = rev.SparkFlex(CoralConstants.kCoralTrackChannel, rev.SparkFlex.MotorType.kBrushless)
        
        # TODO: Can also be wpilib.PneumaticsModuleType.REVPH
        self.leftFlipperPiston = wpilib.Solenoid(wpilib.PneumaticsModuleType.CTREPCM, CoralConstants.kLeftSolenoidChannel)
        self.rightFlipperPiston = wpilib.Solenoid(wpilib.PneumaticsModuleType.CTREPCM, CoralConstants.kRightSolenoidChannel)
        
        # Supposed to give the motor some default settings
        self.motorControllerConfig = rev.SparkMaxConfig()
        self.trackMotor.configure(
            self.motorControllerConfig,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )
        
        # Put breaker light stuff here
        # self.leftBreakerLight = DigitalInput(CoralConstants.kLeftBreakerLight)
        # self.rightBreakerLight = DigitalInput(CoralConstants.kRightBreakerLight)
        
        # Setup the Timer
        self.timer = wpilib.Timer()
        self.timer.start()
        
        # Set up the SmartDashboard variables
        self.setupSmartDashboard()
        
        # self.setupShuffleBoard()
        
    def center(self):
        '''Centers the coral on the track using two breaker lights'''
        
        self.changeFlippers(True)
        
        # Aidan did this yay
        # if self.rightBreakerLight.get() and not self.leftBreakerLight.get(): # Move track left
        #     self.spinTrackMotor(-self.trackCenterMultiplier)
        # elif self.leftBreakerLight.get() and not self.rightBreakerLight.get(): # Move track right
        #     self.spinTrackMotor(self.trackCenterMultiplier)
        # else: # Stop moving track
        #     self.spinTrackMotor(0)
        
    def dischargeButtonPressed(self):
        '''Runs periodically when discharge button pressed'''
        self.discharge() 
        self.timer.reset()
        
        self.updateSmartDashboard() # Keep this at the end of the method
    
    def discharge(self):
        '''Should be doing the discharge when a button is pressed
            "direction" is 1 or -1; indicating the direction the track discharges the coral'''
            
        direction = 1 # Temporary until it can automatically get the direction
        # direction = self.getCorrectDirectionToDischargeCoral()
    
        self.spinTrackMotor(direction * self.trackDischargeMultiplier)
        
        elevatorAtL4 = True # Figure out how to get these two values
        noBreakerLightsDetectingCoral = True
        if elevatorAtL4 and noBreakerLightsDetectingCoral:
            self.changeFlippers(True)
        else:
            self.changeFlippers(False)
        
    def default(self):
        '''When discharge button is not pressed'''
        if self.timer.get() > self.coralDischargeTime:
            beamBrakerLightsAddedToRobot = False
            if beamBrakerLightsAddedToRobot:
                self.center()
            else:
                self.spinTrackMotor(0) # Temporary thing for now
                self.changeFlippers(True)
        else:
            self.discharge()
                
        self.updateSmartDashboard() # Keep this at the end of the method
                
    def spinTrackMotor(self, value):
        '''Function that handles all the motors '''
        self.trackMotor.set(value)
        
    def changeFlippers(self, doesActivate):
        '''Changes the flippers on the track. Active means the flippers are up.'''
        # Do the solenoid stuff here
        # It's "not doesActivate" because pistons retract when flippers are up and expand when flippers are down
        self.leftFlipperPiston.set(not doesActivate)
        self.rightFlipperPiston.set(not doesActivate)
        
    # Below is all the dashboard stuff
        
    def setupSmartDashboard(self) -> None:
        '''Sets up the Smart Dashboard for with all the cool things'''
        SmartDashboard.putNumber("Coral Track Motor", self.trackMotor.get())
        SmartDashboard.putNumber("Left Solenoid", self.leftFlipperPiston.get())
        SmartDashboard.putNumber("Right Solenoid", self.rightFlipperPiston.get())
        # SmartDashboard.putBoolean(" - Left Breaker Light", self.leftBreakerLight.get())
        # SmartDashboard.putBoolean(" - Right Breaker Light", self.rightBreakerLight.get())
        
        SmartDashboard.putNumber("Center Multiplier", self.trackCenterMultiplier)
        SmartDashboard.putNumber("Discharge Multiplier", self.trackDischargeMultiplier)
        SmartDashboard.putNumber("Discharge Time", self.coralDischargeTime)
        # Put more stuff here maybe
        
    def updateSmartDashboard(self) -> None:
        '''Updates the Smart Dashboard for with all the cool things'''
        
        # Update values TO the Smart Dashboard (put stuff here)
        SmartDashboard.putNumber("Coral Track Motor", self.trackMotor.get())
        SmartDashboard.putNumber("Left Solenoid", self.leftFlipperPiston.get())
        SmartDashboard.putNumber("Right Solenoid", self.rightFlipperPiston.get())
        # SmartDashboard.putBoolean(" - Left Breaker Light", self.leftBreakerLight.get())
        # SmartDashboard.putBoolean(" - Right Breaker Light", self.rightBreakerLight.get())
        
        # Update values FROM the Smart Dashboard
        self.trackCenterMultiplier = SmartDashboard.getNumber("Center Multiplier", self.trackCenterMultiplier)
        self.trackDischargeMultiplier = SmartDashboard.getNumber("Discharge Multiplier", self.trackDischargeMultiplier)
        self.coralDischargeTime = SmartDashboard.getNumber("Discharge Time", self.coralDischargeTime)
        
    def setupShuffleBoard(self):
        '''For testing because ShuffleBoard is theoretically cooler'''
        Shuffleboard.getTab("Coral").addPersistent("Discharge Multiplier", self.trackDischargeMultiplier)
        (Shuffleboard.getTab("Coral")
            .addPersistent("Discharge Time", self.coralDischargeTime)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withSize(2, 1) # make the widget 2x1
            .withPosition(4, 3) # place it in the top-left corner
        ) # TODO: Actually this later
        
    def updateShuffleBoard(self):
        '''For testing because ShuffleBoard is theoretically cooler'''
        
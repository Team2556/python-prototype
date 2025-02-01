# What will this module do?

'''

NEO Vortex - Intake
NEO 550 - Extract


* When we feed coral to the robot's claw from our coral station, the claw will receive it sideways by sucking the coral in with one
  of it's motors. It will remain sideways and NOT orientate OR move.

* Then when the robot approaches the coral reef (the thing in the center of the map with pipes sticking up), the elevator will raise
  up for the coral to be placed. The coral part is where this module comes in.

* When the elevator has raised to a coral slot, we will push the coral into the slot at a fixed speed with the other motor
'''
import revlib
import robotpy
import wpilib
from commands2.subsystem import Subsystem
from constants import CoralConstants # Holds the public motorIDs. In a different script so it can easily be changed later instead of changing motors in every script

from wpilib import Spark

class Coral(Subsystem):
    
    def __init__(self):
        self.intakeMotor = Spark(CoralConstants.kCoralIntakeMotorID)
        self.extractMotor = Spark(CoralConstants.kCoralExtractMotorID)

        # intakeButton = control.Control().controller0
        # extractButton = control.Control().controller1
    class Intake():
        '''Thats where robot obtains the coral that is man-fed from the processor'''
        
          # Thats where the robot gets it - Sebastian 2025


        def __init__(self):
           self.Parent = Coral()
           self.intakeMotor = self.Parent.intakeMotor



        def intakeOn(self):
          ''' Turns on intake motor '''
          
          self.intakeMotor.set(1)
          print("Intake motor set value: ", self.intakeMotor.get())


        def intakeOff(self):
          ''' Turns off intake motor '''

          self.intakeMotor.set(0)
          # print("Intake motor set value: ", self.intakeMotor.get())



    class Extract():
      ''' This is where the robot pushes out the coral, ideally into a reef '''

      def __init__(self):
         self.Parent = Coral()
         self.extractMotor = self.Parent.extractMotor
        

      def extractOn(self):
          ''' Turns on the extract motor '''
          
          self.extractMotor.set(1)
          print("Extract motor set value: ",  self.extractMotor.get())

      def extractOff(self):
          ''' Turns off the extract motor '''
          
          self.extractMotor.set(0)
          # print("Extract motor set value: ",  self.extractMotor.get())


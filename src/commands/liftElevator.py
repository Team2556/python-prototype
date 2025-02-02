from commands2 import Command
from wpilib import XboxController
from wpimath.controller import PIDController
from wpimath.units import meters, inches, seconds, metersToInches, inchesToMeters
from phoenix6 import hardware
from constants import ElevatorConstants
from math import pi
import numpy as np
import time

class DriveElevatorCommand(Command):
    def __init__(self,sub_elevator, joystick: XboxController):
        super().__init__()
        self.sub_elevator = sub_elevator
        self.joystick = joystick
        # self.setpoint = self.sub_elevator.setpoint
        self.addRequirements(sub_elevator)
        self.increment_m_per_sec_held = 1.47*.02
        self.previous_time = None


    def execute(self):
        if not self.previous_time:
            self.previous_time = time.time()
        time_delta = np.min([time.time()-self.previous_time, .02])

        right_trigger_value = self.joystick.getRightTriggerAxis()
        left_trigger_value = self.joystick.getLeftTriggerAxis()
        # self.setpoint = self.sub_elevator.elevmotor_left.get_closed_loop_reference().value /(ElevatorConstants.kElevatorGearing/(2*pi*ElevatorConstants.kElevatorDrumRadius))
        if self.joystick.rightTrigger(threshold=.05):
            self.sub_elevator.update_setpoint( self.increment_m_per_sec_held * right_trigger_value * time_delta)
        elif self.joystick.leftTrigger(threshold=.05):
            self.sub_elevator.update_setpoint(- self.increment_m_per_sec_held * left_trigger_value* time_delta)
        self.previous_time = time.time()
        

        if self.joystick.a():
            ( self.sub_elevator.update_setpoint(inchesToMeters(7), incremental=False))
        if self.joystick.b():
            ( self.sub_elevator.update_setpoint(inchesToMeters(21), incremental=False))
        if self.joystick.y():
            ( self.sub_elevator.update_setpoint(inchesToMeters(36), incremental=False))
        
        self.sub_elevator.moveElevator()


    def end(self, interrupted: bool):
        self.sub_elevator.moveElevator(0)
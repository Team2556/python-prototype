import commands2
import wpilib
from wpilib import Timer, XboxController
from wpimath.controller import PIDController
from phoenix6 import hardware, controls
from phoenix6.controls import Follower
from phoenix6.signals import NeutralModeValue


class OneMotor(commands2.PIDSubsystem):
    def __init__(self, motor: list[hardware.TalonFX]):
        super().__init__(
            PIDController(Kp=0, Ki=0, Kd=0, period=0.02),
            initial_position=0
        )
        self.motor = motor[0]
        self.follower = motor[1]
        self.follower.set_control(request=Follower(self.motor.device_id, oppose_master_direction=True))
        self.motor.setNeutralMode(NeutralModeValue.BRAKE)
        self.follower.setNeutralMode(NeutralModeValue.BRAKE)

    def execute(self):
        # print("OneMotor.execute()++++++++++++++++++++++++++++++++++++++++++++++++============================")
        # self.motor.set(-self.joystick.getRightX())
        pass

    def set(self, speed: float):
        self.motor.set(speed)
        
    def end(self):
        self.motor.stopMotor()

    def isFinished(self):
        return False
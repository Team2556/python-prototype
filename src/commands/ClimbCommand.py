from wpilib import XboxController
import commands2
from subsystems.ClimbSubsystem import ClimbSubsystem
from typing import Callable
from constants import ClimbConstants

class ClimbCommand(commands2.Command):
    """
    Command to control the climbing mechanism using button inputs.
    """

    def __init__(self, climb: ClimbSubsystem, deploy: Callable[[], bool], climbdown: Callable[[], bool],  
                 #Armvert: Callable[[], bool], Armdown: Callable[[], bool], 
                 isFullPower: Callable[[], bool],): #joystick : XboxController):
        super().__init__()
        self.climb = climb
        self.deploy = deploy
        self.down = climbdown
        #self.Armvert = Armvert
        #self.Armdown = Armdown
        self.isFullPower = isFullPower
        #self.joystick = joystick

        self.addRequirements(climb)

    def initialize(self, mode):
        """Called when the command is initially scheduled."""
        self.mode = mode

    def execute(self):
        """Called every time the scheduler runs while the command is scheduled."""
        self.speed = ClimbConstants.kSpeed if self.isFullPower() else 0.75 * ClimbConstants.kSpeed

        # Manually controlling climb arm height
        if self.deploy():
            self.climb.setMotorOutputManual(self.isFullPower)
        elif self.down() and not self.climb.getIsBottomLimitSwitchPressed():
            self.climb.setMotorOutputManual(-self.speed)
        else:
            self.climb.setMotorOutputManual(0)

        

        # Manually controlling climb arm angle
      #  if self.Armvert():
        #    self.climb.setClimbVertical()
       # elif self.Armdown():
       #     self.climb.setClimbAngled()

    def end(self, interrupted: bool):
        """Called once the command ends or is interrupted."""
        self.climb.setMotorOutputManual(0)
       

    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        return False
import wpilib
from constants import AlgaeConstants

class AlgaeHandler():
    
    def __init__():
        
        # We don't know the motor channel yet
        motorChannel1 = AlgaeConstants.kIntakeCANAddress
        motorChannel2 = 2
        
        AlgaeIntakeMotor = wpilib.PWMSparkMax(motorChannel1)
        # set() function makes it turn
        
    def cycle():
        ...
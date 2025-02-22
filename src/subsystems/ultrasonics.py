import wpilib
import commands2
from wpilib import SmartDashboard


class Ultrasonics(commands2.Subsystem):# .ProfiledPIDSubsystem):
    def __init__(self) -> None:
        serialPort = wpilib.SerialPort(9600, wpilib.SerialPort.Port.kUSB)
        def readSerialDistance(self):
            if serialPort.getBytesReceived() > 0:
                try:
                    data = serialPort.readString()
                    return float(data)
                except ValueError:
                    return None
            return None

        # print(readSerialDistance())
    
    def periodic(self) -> None:
        # SmartDashboard.putNumber("Elevator Distance", readSerialDistance())
        pass
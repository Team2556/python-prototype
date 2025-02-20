import commands2
import wpilib
from constants import PneumaticConstants


class PneumaticSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.hub = wpilib.PneumaticHub(PneumaticConstants.kHub)
        self.hub.clearStickyFaults()

        self.hub.enableCompressorAnalog(70, 90)

        self.solenoids = [
            (
                self.hub.makeSolenoid(channel)
                if self.hub.checkSolenoidChannel(channel)
                else False
            )
            for channel in range(16)
        ]

    def enableSolenoid(self, channel):
        """Enables a Solenoid based on channel."""
        solenoid = self.solenoids[channel]
        
        if not solenoid:
            print("Solenoid Not Found")
            return

        solenoid.set(True)

    def disableSolenoid(self, channel):
        """Disables a Solenoid based on channel."""
        solenoid = self.solenoids[channel]
        
        if not solenoid:
            print("Solenoid Not Found")
            return

        solenoid.set(False)
    
    def getSolenoid(self, channel: bool):
        """Returns state of Solenoid based on channel."""
        solenoid = self.solenoids[channel]
        
        if not solenoid:
            print("Solenoid Not Found")
            return

        solenoid.get()
    
    def pulseSolenoid(self, channel: bool, duration: float = 0.1):
        """Pulses a Solenoid based on channel."""
        solenoid = self.solenoids[channel]
        
        if not solenoid:
            print("Solenoid Not Found")
            return

        solenoid.setPulseDuration(duration)
        solenoid.startPulse()
        
    def toggleSolenoid(self, channel: bool):
        """Returns state of Solenoid based on channel."""
        solenoid = self.solenoids[channel]
        
        if not solenoid:
            print("Solenoid Not Found")
            return

        solenoid.toggle()
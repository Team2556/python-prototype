import wpilib
import commands2
import rev

from constants import CoralConstants


class CoralTrack(commands2.Subsystem):
    def __init__(self):
        self.motor_controller = rev.SparkFlex(
            CoralConstants.kCoralMotorPort, rev.SparkFlex.MotorType.kBrushless
        )
        self.motor_controller.configure(
            rev.SparkMaxConfig(),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )

        self.left_detector = wpilib.DigitalInput(CoralConstants.kLeftBreakerLight)
        self.center_detector = wpilib.DigitalInput(CoralConstants.kCenterBreakerLight)
        self.right_detector = wpilib.DigitalInput(CoralConstants.kRightBreakerLight)

    def center_coral(self, speed=0.08):
        """Centers Coral on track using left and right sensor"""
        is_Left = self.left_detector.get()
        is_Right = self.right_detector.get()

        if is_Left and not is_Right:
            self.set_motor(speed)
            return

        if is_Right and not is_Left:
            self.set_motor(-speed)
            return
        
        # If Neither or Both detectors return True
        self.disable_motor()

    def set_motor(self, speed):
        """Sets Coral Track motor to a specific speed"""
        self.motor_controller.set(speed)

    def disable_motor(self):
        """Disables Coral Track motor"""
        self.motor_controller.set(0)

    def get_detectors(self):
        """Returns a list of the states of the three sensors"""
        return [
            self.left_detector.get(),
            self.center_detector.get(),
            self.right_detector.get(),
        ]
    
    def detect_coral(self):
        """Returns True if Coral detected on track"""
        return self.left_detector.get() or self.center_detector.get() or self.right_detector.get()
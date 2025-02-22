import wpilib
import commands2


class LEDHandler(commands2.Subsystem):
    def __init__(self, timer: wpilib.Timer):
        super().__init__()
        self.timer = timer
        self.prev_time = 0

        LED_pwm_port = 5  # Placeholder
        self.LED = wpilib.AddressableLED(LED_pwm_port)

        LED_buffer_length = 60
        self.LED_buffer = [
            wpilib.AddressableLED.LEDData() for _ in range(LED_buffer_length)
        ]

        self.LED.setLength(len(self.LED_buffer))

        self.LED_patterns = {
            "red": wpilib.LEDPattern.solid(wpilib.Color.kRed),
            "green": wpilib.LEDPattern.solid(wpilib.Color.kGreen),
            "yellow": wpilib.LEDPattern.solid(wpilib.Color.kYellow),
            "purple": wpilib.LEDPattern.solid(wpilib.Color.kPurple),
            "blue": wpilib.LEDPattern.solid(wpilib.Color.kBlue),
        }
        self.priority_color = [None, 0, 0]
        self.default_color = 'purple'

    def default(self):
        tick_duration = self.timer.get() - self.prev_time
        self.prev_time = self.timer.get()
        
        if self.priority_color[0]:
            self.set_color(self.priority_color[0])

            self.priority_color[2] += tick_duration
            if self.priority_color[2] > self.priority_color[1]:
                self.priority_color = [None, 0, 0]
        else:
            self.set_color(self.default_color)

    def set_priority_color(self, color, duration=1):
        self.priority_color = [color, duration, 0]
    
    def set_default_color(self, color):
        self.default_color = color;

    def set_color(self, color, start=0, end=-1):
        if not color.lower() in self.LED_patterns:
            print('Color Not Found')
            self.set_color('purple')
            return
        
        target_LED_buffer = self.LED_buffer[start:end]
        self.LED_patterns[color.lower()].applyTo(target_LED_buffer)
        self.LED.setData(target_LED_buffer)

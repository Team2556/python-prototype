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
        self.color_backlog = []

    def default(self):
        tick_duration = self.timer.get() - self.prev_time
        self.prev_time = self.timer.get()
        
        if not len(self.color_backlog):
            self.set_color("purple")
            return
        
        self.set_color(self.color_backlog[0][0], self.color_backlog[0][1], self.color_backlog[0][2])
        
        if self.color_backlog[0][-1] > self.color_backlog[0][-2]:
            self.color_backlog.pop(0)
        else:
            self.color_backlog[0][-1] += tick_duration
        

    def que_color(self, color, start=0, end=-1, duration=1, priority=False):
        if priority:
            self.color_backlog.insert(0, [color, start, end, duration, 0])
        else:
            self.color_backlog.append([color, start, end, duration, 0])

    def set_color(self, color, start=0, end=-1):
        target_LED_buffer = self.LED_buffer[start:end]
        self.LED_patterns[color].applyTo(target_LED_buffer)
        self.LED.setData(target_LED_buffer)

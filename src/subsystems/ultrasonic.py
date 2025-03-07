# create a commands2 subsyteem for the ultrasonic sensor 
# the sensor input will be supplied by a RIOduino
# Example code for a standalone python program is below
#import serial
# import time

# # Replace 'COM3' with your Arduino's serial port; find it in the Arduino IDE
# arduino_port = serial.Serial(port='COM4', baudrate=9600, timeout=.1)
# time.sleep(2) # wait a bit for the connection to establish

# while True:
# 	try:
# 		if arduino_port.in_waiting > 0:
# 			data = arduino_port.readline().decode('utf-8').rstrip()
# 			print(data)
# 	except serial.SerialException:
# 		print("Serial connection lost.")
# 		break
# 	except UnicodeDecodeError:
# 		print("Could not decode data")
# 	time.sleep(0.1)


import wpilib
import commands2
from wpilib import AnalogInput, SmartDashboard


class Ultrasonic(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.ultrasonic = AnalogInput(0)
        self.ultrasonic2 = AnalogInput(1)

        
        # '/dev/ttyUSB0'
    #     self.arduino_port = serial.Serial(port='/dev/bus/usb/001', baudrate=9600, timeout=.1)
    #     time.sleep(2) # wait a bit for the connection to establish

    # def get_distance(self):
    #     try:
    #         if self.arduino_port.in_waiting > 0:
    #             data = self.arduino_port.readline().decode('utf-8').rstrip()
    #             return data
    #     except serial.SerialException:
    #         print("Serial connection lost.")
    #     except UnicodeDecodeError:
    #         print("Could not decode data")
    #     return None
    
    def periodic(self):
        distance = self.ultrasonic.getVoltage() * 1000 *.042
        wpilib.SmartDashboard.putNumber("Ultrasonic Distance", float(distance))
        distance2 = self.ultrasonic2.getVoltage() * 1000 *.042
        wpilib.SmartDashboard.putNumber("Ultrasonic Distance 2", float(distance2))


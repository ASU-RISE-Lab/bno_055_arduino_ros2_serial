from string import Template
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String
from bno_055_arduino_ros2_serial.msg import ImuData  # Custome Message with 4 float values
import numpy as np
import time

class Test :
    def __init__(self):
        self.device = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)

        while(1):
            self.send_serial()
            print(time.time(),":",self.read_serial())
            time.sleep(0.02)

    def send_serial(self):
        msg = '1234'.encode()
        self.device.write(msg)

    def read_serial(self):
        msg = String()
        msg = self.device.readline()
        msg = msg.decode('utf-8').strip()
        return msg

if __name__ == '__main__':
    test = Test()
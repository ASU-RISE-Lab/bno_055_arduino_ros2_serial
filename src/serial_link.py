import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String
from bno_055_arduino_ros2_serial.msg import ImuData
import numpy as np

class BNO055_DATA(Node):

    def __init__(self):
        super().__init__('BNO055_DATA')
        self.publisher_ = self.create_publisher(ImuData, 'IMU_Data', 1)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.device = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)

    def timer_callback(self):

        values = ImuData()
        msg = String()
        self.send_serial()
        msg.data = self.read_serial()
        data = msg.data.split(",")

        for i in range(len(data)):
            if data[i] == '' or data[i] == 'nan' :
                data[i] = '0'
        
        if len(data) < 4:
            for i in range(len(data),4):
                data.append('0')

        values.imu1 = float(data[0])
        values.imu2 = float(data[1])
        values.imu3 = float(data[2])
        values.imu4 = float(data[3])

        self.publisher_.publish(values)
        self.get_logger().info('Imu1:"%f" Imu2:"%f" Imu3:"%f" Imu4:"%f"' % (values.imu1, values.imu2, values.imu3, values.imu4))

    def send_serial(self):
        msg = "alpha".encode()
        self.device.write(msg)

    def read_serial(self):
        msg = String()
        msg = self.device.readline()
        msg = msg.decode('utf-8').strip()
        #self.get_logger().info('Publishing: "%s"' % msg)
        return msg

def main(args=None):
    rclpy.init(args=args)

    bno055 = BNO055_DATA()

    rclpy.spin(bno055)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    BNO055_DATA.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String
from bno_055_arduino_ros2_serial.msg import ImuData  # Custome Message with 4 float values
from px4_msgs.msg import VehicleMagnetometer
import numpy as np
import time

class BNO055_DATA(Node):

    def __init__(self):
        super().__init__('BNO055_DATA')

        self.heading = 0.0

        self.publisher_ = self.create_publisher(ImuData, 'IMU_Data', 1)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.device = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)
        self.imu_sub = self.create_subscription(VehicleMagnetometer ,"/fmu/vehicle_magnetometer/out",self.imu_callback,10)
        self.imu_sub

    def imu_callback(self,msg):
        self.mag_x_gauss = msg.magnetometer_ga[0]
        self.mag_y_gauss = msg.magnetometer_ga[1]
        self.mag_z_gauss = msg.magnetometer_ga[2]

        self.heading = np.arctan2(self.mag_y_gauss,self.mag_x_gauss) * 180 / np.pi

        if self.heading < 0:
            self.heading = 360 + self.heading
        if self.heading > 360:
            self.heading = self.heading - 360

        #print("PX4:",self.heading)

    def timer_callback(self):
        
        values = ImuData()
        msg = String()
        self.send_serial()
        msg.data = self.read_serial()
        data = msg.data.split(",")

        for i in range(len(data)):                  # To Convert any Null data to 0
            if data[i] == '' or data[i] == 'nan' :
                data[i] = '0'

        if len(data) < 4:                           # To Create an array with length 4 irrespective of number of IMUs
            for i in range(len(data),4):
                data.append('0')

        values.imu1 = float(data[0])                # Type Casting
        values.imu2 = float(data[1])
        values.imu3 = float(data[2])
        values.imu4 = float(data[3])

        print("PX4",self.heading,"IMU1:",values.imu1,"IMU2:",values.imu2)

        # values.imu1 = values.imu1 - self.heading
        # values.imu2 = values.imu2 - self.heading
        # values.imu3 = values.imu3 - self.heading
        # values.imu4 = values.imu4 - self.heading

        self.publisher_.publish(values)

        # self.get_logger().info('Imu1:"%f" Imu2:"%f" Imu3:"%f" Imu4:"%f"' % (values.imu1, values.imu2, values.imu3, values.imu4))

    def send_serial(self):
        msg = "1234".encode()
        self.device.write(msg)

    def read_serial(self):
        msg = String()
        msg = self.device.readline()
        msg = msg.decode('utf-8').strip()
        while self.device.in_waiting > 0: # To Flush out the input buffer to avoid overflow
            self.device.readline()
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

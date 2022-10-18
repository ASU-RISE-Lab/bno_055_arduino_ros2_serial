import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String
from bno_055_arduino_ros2_serial.msg import ImuData  # Custome Message with 4 float values
from px4_msgs.msg import VehicleMagnetometer
import numpy as np
import time
from scipy.spatial.transform import Rotation
import math
from px4_msgs.msg import VehicleOdometry

class BNO055_DATA(Node):

    def __init__(self):
        super().__init__('BNO055_DATA')

        self.heading = 0.0
        self.inital_yaw = 0.0
        self.initial_yaw_set = False
        self.actual_yaw = 0.0

        self.publisher_ = self.create_publisher(ImuData, 'IMU_Data', 1)
        timer_period = .02 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.device = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)
        self.odom_sub = self.create_subscription(VehicleOdometry ,"/fmu/vehicle_odometry/out",self.odom_callback,10)

        self.q = np.zeros(4)

    def odom_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

        self.q[3] = self.qw = msg.q[0]
        self.q[0] = self.qx = msg.q[1]
        self.q[1] = self.qy = msg.q[2]
        self.q[2] = self.qz = msg.q[3]

        self.rot = Rotation.from_quat(self.q)
        self.rot_euler = self.rot.as_euler('xyz', degrees=True)  # [roll, pitch, yaw]
        #self.yaw_curr = self.rot_euler[2] * math.pi / 180.0
        #print(self.rot_euler[2])
        if(self.initial_yaw_set == False):
            self.inital_yaw = self.rot_euler[2]
            self.initial_yaw_set = True

        self.actual_yaw = self.rot_euler[2] - self.inital_yaw
        #print("Actual Yaw: ", self.actual_yaw)

    def timer_callback(self):
        
        values = ImuData()
        msg = String()
        self.send_serial()
        msg.data = self.read_serial()
        data = msg.data.split(",")

        print(data)

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
        
        # im1_value = values.imu1

        # if(abs(values.imu1) > 180.0):
        #     values.imu1 = values.imu1 - values.imu1/abs(values.imu1)*360

        # im2_value = values.imu2

        # if(abs(values.imu2) > 180.0):
        #     values.imu2 = values.imu2 - values.imu2/abs(values.imu2)*360

        # values.imu1 = values.imu1 - 45.0
        # values.imu2 = values.imu2 + 45.0

        # self.arm1_delta = values.imu1 - (self.actual_yaw)
        # self.arm2_delta = values.imu2 - (self.actual_yaw)


        # #print(self.actual_yaw)
        # #print("Arm1-raw: ",im1_value, "Arm1_cal", values.imu1, "Arm1-diff: ", self.arm1_delta)
        # print("Arm1-dx: ", self.arm1_delta, "Arm2-dx: ", self.arm2_delta)

        # #print("PX4",self.heading,"IMU1:",values.imu1,"IMU2:",values.imu2)

        # # values.imu1 = values.imu1 - self.heading
        # # values.imu2 = values.imu2 - self.heading
        # # values.imu3 = values.imu3 - self.heading
        # # values.imu4 = values.imu4 - self.heading

        # self.publisher_.publish(values)

        # # self.get_logger().info('Imu1:"%f" Imu2:"%f" Imu3:"%f" Imu4:"%f"' % (values.imu1, values.imu2, values.imu3, values.imu4))

    def send_serial(self):
        msg = "1234".encode()
        self.device.write(msg)

    def read_serial(self):
        msg = String()
        msg = self.device.readline()
        msg = msg.decode('utf-8').strip()
        #self.device.flushInput()
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

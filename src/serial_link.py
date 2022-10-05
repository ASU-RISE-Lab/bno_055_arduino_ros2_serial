# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String



class BNO055_DATA(Node):

    def __init__(self):
        super().__init__('BNO055_DATA')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.device = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)

    def timer_callback(self):
        msg = String()
        self.send_serial()
        msg.data = self.read_serial()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

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

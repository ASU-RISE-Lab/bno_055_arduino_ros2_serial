import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String



class BNO055_DATA(Node):

    def __init__(self):
        super().__init__('BNO055_DATA')
        self.publisher_ = self.create_publisher(String, 'topic', 1)
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

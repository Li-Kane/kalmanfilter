import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class SimpleRepublisher(Node):

    def __init__(self):
        super().__init__('simple_republisher')
        self.republisher = self.create_publisher(Float32MultiArray, 'predicted', 10)
        self.low_freq_sub = self.create_subscription(Float32MultiArray, 'low_freq', self.low_freq_callback, 10)

    def low_freq_callback(self, msg):
        self.republisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    simple_republisher = SimpleRepublisher()

    rclpy.spin(simple_republisher)

    simple_republisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

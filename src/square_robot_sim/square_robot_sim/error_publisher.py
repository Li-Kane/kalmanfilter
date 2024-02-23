import time
import numpy as np
from numpy.linalg import norm

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

class ErrorPublisher(Node):

    def __init__(self):
        super().__init__('error_publisher')
        self.error_publisher = self.create_publisher(Float32, 'error', 10)
        self.high_freq_sub = self.create_subscription(Float32MultiArray, 'high_freq', self.error_callback, 10)
        self.predicted_sub = self.create_subscription(Float32MultiArray, 'predicted', self.predicted_freq_callback, 10)
        self.last_seen_prediction = None
        self.has_seen_prediction = False
        self.last_time_prediction_seen = None

    def predicted_freq_callback(self, msg):
        self.has_seen_prediction = True
        self.last_time_prediction_seen = time.time()
        self.last_seen_prediction = [msg.data[0], msg.data[1]]

    def error_callback(self, msg):
        if not self.has_seen_prediction or self.last_seen_prediction is None or time.time() - self.last_time_prediction_seen > 1:
            return
        error = norm(np.array([msg.data[0], msg.data[1]]) - np.array(self.last_seen_prediction))
        error_msg = Float32()
        error_msg.data = error
        self.error_publisher.publish(error_msg)


def main(args=None):
    rclpy.init(args=args)

    error_publisher = ErrorPublisher()

    rclpy.spin(error_publisher)

    error_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

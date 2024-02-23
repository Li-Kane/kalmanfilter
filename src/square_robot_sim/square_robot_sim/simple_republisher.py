import rclpy
from rclpy.node import Node
import filterpy
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

from std_msgs.msg import Float32MultiArray

def filter():
    tracker = KalmanFilter(dim_x=6, dim_z=2)
    dt = 1/10

    tracker.F = np.array([[1, dt, 0.5*dt**2, 0, 0, 0],
                          [0, 1, dt, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 1, dt, 0.5*dt**2],
                          [0, 0, 0, 0, 1, dt],
                          [0, 0, 0, 0, 0, 1]])

    tracker.Q = Q_discrete_white_noise(dim=3, dt=dt, var=0.001, block_size=2)

    tracker.H = np.array([[1,0,0,0,0,0],
                          [0,0,0,1,0,0]])

    tracker.R = np.array([[16, 0],
                          [0, 16]])

    tracker.x = np.array([[0,0,0,0,0,0]]).T

    tracker.P = np.array([[16, 0, 0, 0, 0, 0],
                          [0, 200, 0, 0, 0, 0],
                          [0, 0, 100, 0, 0, 0],
                          [0, 0, 0, 16, 0, 0],
                          [0, 0, 0, 0, 200, 0],
                          [0, 0, 0, 0, 0, 100]])
    return tracker

class SimpleRepublisher(Node):

    def __init__(self):
        super().__init__('simple_republisher')
        self.republisher = self.create_publisher(Float32MultiArray, 'predicted', 10)
        self.low_freq_sub = self.create_subscription(Float32MultiArray, 'low_freq', self.low_freq_callback, 10)
        self.filter_pub = self.create_publisher(Float32MultiArray, 'filter', 10)
        self.filter = filter()

    def low_freq_callback(self, msg):
        self.republisher.publish(msg)
        self.filter.predict()
        self.filter.update([msg.data[0], msg.data[1]])
        prediction = self.filter.x
        prediction = prediction.tolist()
        print(prediction[0])
        print(type(prediction[0]))
        prediction_msg = Float32MultiArray()
        prediction_msg.data = [prediction[0], prediction[1]]
        self.filter_pub.publish(prediction_msg)

def main(args=None):
    rclpy.init(args=args)

    simple_republisher = SimpleRepublisher()

    rclpy.spin(simple_republisher)

    simple_republisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

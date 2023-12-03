import time
from typing import List

import pybullet as p
import pybullet_data

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SquareRobotNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.do_pybullet_setup()
        p.setRealTimeSimulation(1, physicsClientId=self.physicsClient)
        self.starttime = time.time()
        self.i = 1
        self.high_freq_pub = self.create_publisher(Float32MultiArray, 'high_freq', 10)
        self.low_freq_pub = self.create_publisher(Float32MultiArray, 'low_freq', 10)
        self.timer = self.create_timer((1 / 120), self.do_pybullet_stuff)
        self.timer = self.create_timer(.1, self.low_freq_timer_callback)

    def do_pybullet_setup(self):
        self.physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.physicsClient)  # optionally
        p.setGravity(0, 0, -9.81, physicsClientId=self.physicsClient)
        planeId = p.loadURDF("plane.urdf", physicsClientId=self.physicsClient)
        startPos = [0, 0, 1]
        start_or = p.getQuaternionFromEuler([0, 0, 0])
        self.robotid = p.loadURDF("/home/michael/Documents/kfenv/src/square_robot_sim/square_robot_sim/slidingbox.urdf",
                                  startPos, start_or, useFixedBase=True, useMaximalCoordinates=False,
                                  physicsClientId=self.physicsClient)

        p.resetJointState(self.robotid, 0, 2, physicsClientId=self.physicsClient)
        p.resetJointState(self.robotid, 1, 0, physicsClientId=self.physicsClient)

    def compute_desired_joint_pos(self, time: float) -> List[float]:
        """Every second, move the joint in the box which is centered at [0,0] and extends by 2 in
        both directions"""
        time = time % 4
        if time < 1:
            return [2, 0]
        elif time < 2:
            return [0, 2]
        elif time < 3:
            return [-2, 0]
        else:
            return [0, -2]

    def get_vels(self) -> List[float]:
        # joint_states = p.getJointStates(self.robotid, [0, 1], physicsClientId=self.physicsClient)
        # joint0_state = joint_states[0]
        # joint1_state = joint_states[1]
        # joint0_vel = joint0_state[1]
        # joint1_vel = joint1_state[1]
        # return [joint0_vel, joint1_vel]
        # get velocity of link 1 (the box)
        link_states = p.getLinkState(self.robotid, 1, computeLinkVelocity=1, physicsClientId=self.physicsClient)
        link_vel = link_states[6]
        return [link_vel[0], link_vel[1]]

    def do_pybullet_stuff(self):
        joint0, joint1 = self.compute_desired_joint_pos((time.time() - self.starttime) / 2)
        p.setJointMotorControl2(self.robotid, 0, p.POSITION_CONTROL, targetPosition=joint0, maxVelocity=4.0, force=60,
                                physicsClientId=self.physicsClient)
        p.setJointMotorControl2(self.robotid, 1, p.POSITION_CONTROL, targetPosition=joint1, maxVelocity=4.0, force=60,
                                physicsClientId=self.physicsClient)
        self.high_freq_pub.publish(Float32MultiArray(data=self.get_vels()))

    def low_freq_timer_callback(self):
        self.low_freq_pub.publish(Float32MultiArray(data=self.get_vels()))


def main(args=None):
    rclpy.init(args=args)

    node = SquareRobotNode("square_robot_node")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

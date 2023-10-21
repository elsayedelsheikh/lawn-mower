#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import degrees
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointTrajectoryControllerTester(Node):
    def __init__(self):
        super().__init__('joint_trajectory_controller_tester')
        self.state_sub_ = self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_position_controller/state',
            self.state_callback,
            10
        )

        self.traj_pub_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_position_controller/joint_trajectory',
            10
        )

        self.timer_ = self.create_timer(0.1, self.timer_callback)

        ## Variables
        self.joint_names = []
        self.pan_angle = 0.0
        self.tilt_angle = 0.0
    
    def state_callback(self, msg):
        self.joint_names = msg.joint_names
        self.pan_angle = msg.actual.positions[0]
        self.tilt_angle =msg.actual.positions[1]
        self.get_logger().info('Pan: %0.2f' % degrees(self.pan_angle))
        self.get_logger().info('Tilt: %0.2f' % degrees(self.tilt_angle))

    def timer_callback(self):
        new_pan = self.pan_angle - 0.1
        new_tilt = self.tilt_angle - 0.1

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.header.stamp = self.get_clock().now().to_msg()

        point = JointTrajectoryPoint()
        point.positions = [new_pan, new_tilt]
        # point.velocities = [-1.0, -1.0]
        # point.accelerations = [0.5, 0.0]
        point.time_from_start.sec = 1

        msg.points.append(point)
        self.traj_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryControllerTester()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
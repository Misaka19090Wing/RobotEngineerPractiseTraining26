#!/usr/bin/env python3
"""
Custom diff drive controller: subscribes to /cmd_vel, computes wheel velocities,
publishes to Gazebo joint velocity topics via ros_gz_bridge.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math


class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')

        # Vehicle parameters
        self.wheel_radius = 0.015       # 15mm wheel radius (30mm diameter)
        self.wheel_separation = 0.148   # distance between left and right wheels along Y

        # Subscriber
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # Publishers for each wheel joint velocity (ROS topics bridged to Gazebo)
        self.pub_lf = self.create_publisher(
            Float64, '/model/autoVehicle/joint/left_front_joint/cmd_vel', 10)
        self.pub_lr = self.create_publisher(
            Float64, '/model/autoVehicle/joint/left_rear_joint/cmd_vel', 10)
        self.pub_rf = self.create_publisher(
            Float64, '/model/autoVehicle/joint/right_front_joint/cmd_vel', 10)
        self.pub_rr = self.create_publisher(
            Float64, '/model/autoVehicle/joint/right_rear_joint/cmd_vel', 10)

        self.get_logger().info('DiffDriveController ready')
        self.get_logger().info(f'  wheel_radius={self.wheel_radius}, wheel_separation={self.wheel_separation}')

    def cmd_vel_cb(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        # Kinematics: v_left  = (linear - angular * sep/2) / radius
        #             v_right = (linear + angular * sep/2) / radius
        half_sep = self.wheel_separation / 2.0
        v_left  = (linear - angular * half_sep) / self.wheel_radius
        v_right = (linear + angular * half_sep) / self.wheel_radius

        # Publish to all four wheels
        cmd_left = Float64(data=v_left)
        cmd_right = Float64(data=v_right)

        self.pub_lf.publish(cmd_left)
        self.pub_lr.publish(cmd_left)
        self.pub_rf.publish(cmd_right)
        self.pub_rr.publish(cmd_right)


def main():
    rclpy.init()
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

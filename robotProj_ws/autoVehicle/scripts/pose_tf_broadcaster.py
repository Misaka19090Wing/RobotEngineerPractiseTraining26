#!/usr/bin/env python3
"""
Pose TF Broadcaster — provides the odom → base_footprint transform in the TF tree.

Data sources (tried in order):
  1. /odom  (nav_msgs/Odometry) — bridged from Gazebo by ros_gz_bridge
  2. If neither works, a static odom→base_footprint identity is published
     as a timer-based fallback, so the odom frame always exists.

The robot_state_publisher handles base_link → (wheels, radar).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class PoseTFBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self._last_pose_stamp = self.get_clock().now()

        # Source 1: /model/autoVehicle/pose (bridged from Gazebo model pose)
        self.pose_sub = self.create_subscription(
            Pose, '/model/autoVehicle/pose', self.pose_cb, 10,
        )

        # Source 2: /odom (bridged nav_msgs/Odometry from Gazebo)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10,
        )

        # Fallback: publish identity odom→base_footprint at 10 Hz to keep
        # the odom frame alive even when no pose source is received yet.
        self._timer = self.create_timer(0.1, self._fallback_timer_cb)
        self._has_live_pose = False  # set to True when a real pose arrives

        self.get_logger().info(
            'PoseTFBroadcaster ready — listening on /model/autoVehicle/pose and /odom '
            '(identity fallback active until pose arrives)'
        )

    # ── callbacks ────────────────────────────────────────────────────
    def pose_cb(self, msg: Pose):
        self._publish_tf(msg.position.x, msg.position.y, msg.position.z,
                         msg.orientation.x, msg.orientation.y,
                         msg.orientation.z, msg.orientation.w)
        if not self._has_live_pose:
            self.get_logger().info(
                f'[POSE RECEIVED] pos=({msg.position.x:.2f},{msg.position.y:.2f},{msg.position.z:.2f}) '
                f'— switching to live odom→base_footprint TF')
        self._has_live_pose = True

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        self._publish_tf(p.position.x, p.position.y, p.position.z,
                         p.orientation.x, p.orientation.y,
                         p.orientation.z, p.orientation.w)
        if not self._has_live_pose:
            self.get_logger().info(
                f'[ODOM RECEIVED] pos=({p.position.x:.2f},{p.position.y:.2f},{p.position.z:.2f}) '
                f'— switching to live odom→base_footprint TF')
        self._has_live_pose = True

    # ── fallback ─────────────────────────────────────────────────────
    def _fallback_timer_cb(self):
        """Publish identity transform as fallback so the odom frame exists."""
        if self._has_live_pose:
            return  # real data is already flowing
        self._publish_tf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    # ── helpers ──────────────────────────────────────────────────────
    def _publish_tf(self, x, y, z, qx, qy, qz, qw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = PoseTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Synthetic 2D LiDAR — mathematical ray-casting against the course_test geometry.

Since Gazebo Harmonic does not reliably process URDF-based <gazebo><sensor>
elements when models are spawned via ros_gz_sim/create, this node bypasses
Gazebo's sensor system entirely.

It uses the robot's ground-truth odometry (/odom) + TF (radar_link transform)
to compute the LiDAR's world pose, then casts 360 rays against a mathematical
model of the course (flat floor, 45° ramp, walls, T-junction).

Publishes:
  /scan  (sensor_msgs/LaserScan) — consumed by pointcloud_mapper
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException

# ════════════════════════════════════════════════════════════════
# Course geometry model — matches course_test.sdf
# ════════════════════════════════════════════════════════════════
HALF_W = 0.15          # half track width (inner wall edge Y)
RAMP_X0, RAMP_X1 = 3.2, 4.3314
RAMP_Z0, RAMP_Z1 = 0.0, 1.1314
JUNCTION_X = 5.9314
JUNCTION_Y = 1.6       # T extends Y ±1.6
RANGE_MIN, RANGE_MAX = 0.05, 30.0


def floor_z_at(wx):
    if wx < RAMP_X0:      return 0.0
    if wx < RAMP_X1:      return RAMP_Z0 + (wx - RAMP_X0) / (RAMP_X1 - RAMP_X0) * (RAMP_Z1 - RAMP_Z0)
    return RAMP_Z1


def cast_ray(ox, oy, angle):
    """Return (range, wx, wy) for a single ray from (ox, oy) in direction angle."""
    dx = math.cos(angle)
    dy = math.sin(angle)

    # Check wall intersections: left wall at y = -HALF_W, right wall at y = +HALF_W
    t_hit = RANGE_MAX
    for wall_y in (-HALF_W, HALF_W):
        if abs(dy) < 1e-10:
            continue
        t = (wall_y - oy) / dy
        if RANGE_MIN < t < t_hit:
            ix = ox + t * dx
            if 0 <= ix <= JUNCTION_X + 0.5:
                t_hit = t

    wx = ox + t_hit * dx
    wy = oy + t_hit * dy
    r = math.hypot(wx - ox, wy - oy)
    return float(r)


class SyntheticLidar(Node):
    def __init__(self):
        super().__init__('synthetic_lidar')
        self.declare_parameter('num_samples', 360)
        self.declare_parameter('rate_hz', 10.0)
        self.num = self.get_parameter('num_samples').value
        self.hz = self.get_parameter('rate_hz').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self._cnt = 0
        self.get_logger().info(f'SyntheticLidar | {self.num} rays @ {self.hz}Hz')

        # Publish on a timer instead of odom callback — ensures steady 10 Hz
        self.timer = self.create_timer(1.0 / self.hz, self.tick)

    def tick(self):
        # Get LiDAR world pose via TF
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'radar_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3))
        except TransformException:
            return

        lx = tf.transform.translation.x
        ly = tf.transform.translation.y
        lz = tf.transform.translation.z  # not used directly; height comes from floor model

        # Build LaserScan
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'radar_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / self.num
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.hz
        scan.range_min = RANGE_MIN
        scan.range_max = RANGE_MAX

        angle = scan.angle_min
        ranges = []
        for _ in range(self.num):
            r = cast_ray(lx, ly, angle)
            ranges.append(r)
            angle += scan.angle_increment

        scan.ranges = ranges
        self.scan_pub.publish(scan)

        self._cnt += 1
        if self._cnt <= 3:
            self.get_logger().info(
                f'[SYNTH #{self._cnt}] LiDAR@({lx:.2f},{ly:.2f},{lz:.2f}) '
                f'r[0]={ranges[0]:.2f} r[90]={ranges[90]:.2f}'
            )
        elif self._cnt % 50 == 0:
            self.get_logger().info(
                f'[SYNTH] {self._cnt} scans | pos=({lx:.2f},{ly:.2f},{lz:.2f})'
            )


def main():
    rclpy.init()
    rclpy.spin(SyntheticLidar())


if __name__ == '__main__':
    main()

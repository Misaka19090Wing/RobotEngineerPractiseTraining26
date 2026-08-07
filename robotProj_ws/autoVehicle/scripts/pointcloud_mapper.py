#!/usr/bin/env python3
"""
3D Point Cloud Mapper — accumulates LiDAR data into a global map,
publishes the map for RViz2, and saves as PCD on demand.

Inputs:
  /lidar_points  (sensor_msgs/PointCloud2) — 3D LiDAR point cloud
  /scan          (sensor_msgs/LaserScan)    — 2D laser scan (fallback)

Outputs:
  /pointcloud_map (sensor_msgs/PointCloud2) — accumulated map in odom frame

Services:
  /save_map (std_srvs/Trigger) — save map as .pcd
"""
import math
import struct
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from sensor_msgs_py import point_cloud2 as pc2
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener, TransformException


class PointCloudMapper(Node):
    def __init__(self):
        super().__init__('pointcloud_mapper')

        self.map_points = []

        self.declare_parameter('downsample_input', 1)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('save_dir', os.path.expanduser('~/pointcloud_maps'))

        self.downsample_input = self.get_parameter('downsample_input').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.save_dir = self.get_parameter('save_dir').value
        os.makedirs(self.save_dir, exist_ok=True)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers — two data sources, unified accumulation
        self.pc_sub = self.create_subscription(
            PointCloud2, '/lidar_points', self.pc_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self._pc_counter = 0

        # Publisher — accumulated global map (throttled)
        self.map_pub = self.create_publisher(PointCloud2, '/pointcloud_map', 10)
        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.publish_map_callback)

        # Save service
        self.save_srv = self.create_service(
            Trigger, '/save_map', self.save_map_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info(
            f'PointCloudMapper ready (PC2 + LaserScan) — '
            f'publish_rate={self.publish_rate_hz} Hz, save_dir={self.save_dir}'
        )

    # ── PointCloud2 callback (3D LiDAR) ──────────────────────────────
    def pc_callback(self, msg: PointCloud2):
        self._pc_counter += 1
        if self._pc_counter % self.downsample_input != 0:
            return

        tf = self._lookup_transform(msg.header.frame_id)
        if tf is None:
            return
        tx, ty, tz, r00, r01, r02, r10, r11, r12, r20, r21, r22 = tf

        for pt in pc2.read_points(msg, skip_nans=True):
            px, py, pz = pt[0], pt[1], pt[2]
            intensity = pt[3] if len(pt) > 3 else 1.0
            wx = r00*px + r01*py + r02*pz + tx
            wy = r10*px + r11*py + r12*pz + ty
            wz = r20*px + r21*py + r22*pz + tz
            self.map_points.append((wx, wy, wz, intensity))

    # ── LaserScan callback (2D LiDAR fallback) ──────────────────────
    def scan_callback(self, msg: LaserScan):
        tf = self._lookup_transform(msg.header.frame_id)
        if tf is None:
            return
        tx, ty, tz, r00, r01, r02, r10, r11, r12, r20, r21, r22 = tf

        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Point in sensor frame (2D plane, Z=0)
                px = r * math.cos(angle)
                py = r * math.sin(angle)
                pz = 0.0
                # Transform to odom
                wx = r00*px + r01*py + r02*pz + tx
                wy = r10*px + r11*py + r12*pz + ty
                wz = r20*px + r21*py + r22*pz + tz
                self.map_points.append((wx, wy, wz, 1.0))
            angle += msg.angle_increment

    # ── TF lookup helper ────────────────────────────────────────────
    def _lookup_transform(self, from_frame: str):
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', from_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as e:
            self.get_logger().debug(f'TF [{from_frame}→odom] failed: {e}')
            return None

        tr = tf.transform.translation
        rt = tf.transform.rotation
        tx, ty, tz = tr.x, tr.y, tr.z
        qx, qy, qz, qw = rt.x, rt.y, rt.z, rt.w

        # Rotation matrix from quaternion
        xx, yy, zz = qx*qx, qy*qy, qz*qz
        xy, xz, yz = qx*qy, qx*qz, qy*qz
        wx, wy, wz = qw*qx, qw*qy, qw*qz

        return (tx, ty, tz,
                1.0-2.0*(yy+zz), 2.0*(xy-wz),     2.0*(xz+wy),
                2.0*(xy+wz),     1.0-2.0*(xx+zz),  2.0*(yz-wx),
                2.0*(xz-wy),     2.0*(yz+wx),      1.0-2.0*(xx+yy))

    # ── Publish accumulated map ─────────────────────────────────────
    def publish_map_callback(self):
        if not self.map_points:
            return

        pts = self.map_points
        if len(pts) > 100000:
            step = max(1, len(pts) // 100000)
            pts = pts[::step]

        header = rclpy.time.Time().to_msg()
        header.frame_id = 'odom'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        pc_msg = PointCloud2()
        pc_msg.header = header
        pc_msg.height = 1
        pc_msg.width = len(pts)
        pc_msg.fields = fields
        pc_msg.is_bigendian = False
        pc_msg.point_step = 16
        pc_msg.row_step = 16 * len(pts)
        pc_msg.is_dense = True
        pc_msg.data = b''.join(struct.pack('<ffff', x, y, z, i) for (x, y, z, i) in pts)

        self.map_pub.publish(pc_msg)

    # ── Save map service ────────────────────────────────────────────
    def save_map_callback(self, request, response):
        if not self.map_points:
            response.success = False
            response.message = 'No points accumulated yet.'
            return response

        pts = self.map_points
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.save_dir, f'pointcloud_map_{timestamp}.pcd')

        try:
            with open(filename, 'w') as f:
                f.write('# .PCD v0.7 - Point Cloud Data file format\n')
                f.write('VERSION 0.7\nFIELDS x y z intensity\n')
                f.write('SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n')
                f.write(f'WIDTH {len(pts)}\nHEIGHT 1\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                f.write(f'POINTS {len(pts)}\nDATA ascii\n')
                for x, y, z, i in pts:
                    f.write(f'{x:.6f} {y:.6f} {z:.6f} {i:.4f}\n')
            response.success = True
            response.message = f'Saved {len(pts)} points to {filename}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed: {e}'

        return response


def main():
    rclpy.init()
    node = PointCloudMapper()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

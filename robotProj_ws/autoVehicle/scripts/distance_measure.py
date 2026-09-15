#!/usr/bin/env python3
"""
Distance measurement node for the bridge-tray measuring robot.

Measures, in real time, the geometry the robot cares about while driving a
cable tray:

  * forward clearance  — nearest obstacle inside the robot's forward lane
  * left / right wall  — nearest wall on each side, from the +/-90 deg sectors
  * clear width        — left + right (the sensor's lateral offset cancels out)
  * travelled mileage  — path-integrated /odom, i.e. how far the robot has driven
  * tray length        — extent of the accumulated point cloud along its
                         principal (2D PCA) axis, i.e. how long the tray is

All ranges are measured in the LiDAR frame (radar_link), matching the
convention already used by waypoint_navigator._scan_avoidance. radar_link is
mounted with zero rotation relative to base_link, so a scan angle of 0 is the
robot's forward direction.

Topics:
  /distance/forward      (std_msgs/Float32)  forward clearance [m]
  /distance/left         (std_msgs/Float32)  left wall range [m]
  /distance/right        (std_msgs/Float32)  right wall range [m]
  /distance/width        (std_msgs/Float32)  left + right clear width [m]
  /distance/traveled     (std_msgs/Float32)  path-integrated mileage [m]
  /distance/tray_length  (std_msgs/Float32)  measured tray length [m]
  /distance/status       (std_msgs/String)   one-line human readable summary
  /distance_markers      (visualization_msgs/MarkerArray) RViz overlay

Services:
  /distance/status  (std_srvs/Trigger)  current summary
  /distance/reset   (std_srvs/Trigger)  zero mileage / statistics / samples
  /distance/save    (std_srvs/Trigger)  write JSON report + CSV series
"""
import json
import math
import os
import struct
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Float32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_REPORT_DIR = os.path.expanduser('~/distance_reports')

# radar_link offset within base_link, same constants as waypoint_navigator.py
RADAR_X = 0.063205
RADAR_Y = 0.015705

# marker namespaces
NS_TEXT = 'distance_text'
NS_RAYS = 'distance_rays'
NS_SPAN = 'distance_span'
NS_ENDS = 'distance_ends'

# markers are refreshed at publish_rate_hz; a short lifetime makes stale
# namespaces disappear on their own without a DELETEALL sweep
MARKER_LIFETIME = Duration(sec=1)


def _yaw_from_quaternion(q):
    """Extract yaw from a geometry_msgs quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _fmt(value, unit='m', digits=2):
    """Format an optional measurement, using '--' when it is unavailable."""
    if value is None or not math.isfinite(value):
        return '--'
    return f'{value:.{digits}f}{unit}'


def _point(x, y, z):
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def _color(r, g, b, a):
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


class DistanceMeasure(Node):
    def __init__(self):
        super().__init__('distance_measure')

        self.declare_parameter('report_dir', DEFAULT_REPORT_DIR)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('map_rate_hz', 0.5)
        self.declare_parameter('sample_rate_hz', 2.0)
        self.declare_parameter('forward_lane_half_width', 0.10)
        self.declare_parameter('side_fov_deg', 30.0)
        self.declare_parameter('max_map_points', 40000)
        self.declare_parameter('max_samples', 20000)
        self.declare_parameter('min_travel_step', 1e-4)
        self.declare_parameter('marker_z_offset', 0.45)
        self.declare_parameter('near_distance', 0.25)
        self.declare_parameter('warn_distance', 0.60)

        self.report_dir = os.path.expanduser(self.get_parameter('report_dir').value)
        self.lane_half = self.get_parameter('forward_lane_half_width').value
        self.side_half = math.radians(
            self.get_parameter('side_fov_deg').value) / 2.0
        self.max_map_points = int(self.get_parameter('max_map_points').value)
        self.max_samples = int(self.get_parameter('max_samples').value)
        self.min_travel_step = self.get_parameter('min_travel_step').value
        self.marker_z_offset = self.get_parameter('marker_z_offset').value
        self.near_distance = self.get_parameter('near_distance').value
        self.warn_distance = self.get_parameter('warn_distance').value

        # A measurement node must keep measuring even when its report directory
        # is unavailable; only /distance/save is disabled in that case.
        self.report_dir_ok = True
        try:
            os.makedirs(self.report_dir, exist_ok=True)
        except OSError as exc:
            self.report_dir_ok = False
            self.get_logger().warn(
                f'Report directory {self.report_dir} is not writable ({exc}); '
                f'/distance/save will be disabled')

        # ── measurement state ────────────────────────────────────────────────
        self._odom = None           # (x, y, z, yaw) in odom
        self._scan = None
        self._map_msg = None
        self._last_xy = None        # previous odom (x, y) for mileage integration
        self._tick = 0

        self.mileage = 0.0
        self.forward = None
        self.left = None
        self.right = None
        self.width = None
        self.forward_angle = None
        self.left_angle = None
        self.right_angle = None

        # tray extent measured from the accumulated point cloud
        self.map_length = None
        self.map_axis = None
        self.tray_start = None      # (x, y) in odom
        self.tray_end = None

        # statistics + recorded series
        self.width_min = None
        self.width_max = None
        self.width_sum = 0.0
        self.width_count = 0
        self.forward_min = None
        self.samples = []

        # ── ROS interfaces ───────────────────────────────────────────────────
        self.forward_pub = self.create_publisher(Float32, '/distance/forward', 10)
        self.left_pub = self.create_publisher(Float32, '/distance/left', 10)
        self.right_pub = self.create_publisher(Float32, '/distance/right', 10)
        self.width_pub = self.create_publisher(Float32, '/distance/width', 10)
        self.traveled_pub = self.create_publisher(Float32, '/distance/traveled', 10)
        self.tray_pub = self.create_publisher(Float32, '/distance/tray_length', 10)
        self.status_pub = self.create_publisher(String, '/distance/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/distance_markers', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)
        self.map_sub = self.create_subscription(
            PointCloud2, '/pointcloud_map', self.map_cb, 10)

        self.status_srv = self.create_service(
            Trigger, '/distance/status', self.status_cb)
        self.reset_srv = self.create_service(
            Trigger, '/distance/reset', self.reset_cb)
        self.save_srv = self.create_service(
            Trigger, '/distance/save', self.save_cb)

        rate = self.get_parameter('publish_rate_hz').value
        sample_hz = max(self.get_parameter('sample_rate_hz').value, 1e-6)
        self.sample_every = max(1, int(round(rate / sample_hz)))
        self.tick_timer = self.create_timer(1.0 / rate, self.tick)
        self.map_timer = self.create_timer(
            1.0 / max(self.get_parameter('map_rate_hz').value, 1e-6),
            self.measure_tray)

        self.get_logger().info(
            f'DistanceMeasure ready | forward_lane=+/-{self.lane_half:.2f}m '
            f'side_fov={math.degrees(self.side_half) * 2:.0f}deg '
            f'reports={self.report_dir}')

    # ── subscriptions ────────────────────────────────────────────────────────
    def odom_cb(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        self._odom = (x, y, pose.position.z, _yaw_from_quaternion(pose.orientation))

        if self._last_xy is None:
            self._last_xy = (x, y)
            return
        step = math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)
        if step >= self.min_travel_step:
            self.mileage += step

    def scan_cb(self, msg):
        self._scan = msg

    def map_cb(self, msg):
        self._map_msg = msg

    # ── measurement ──────────────────────────────────────────────────────────
    def _sector_min(self, scan, center, half):
        """Nearest valid range within `half` radians of `center`, plus its angle.

        The scan is uniformly spaced, so the angular window maps directly onto
        an index window. Ranges outside [range_min, range_max] are rejected:
        synthetic_lidar.py emits NaN for a miss, a real LD06 reports 0.0.
        """
        count = len(scan.ranges)
        increment = scan.angle_increment
        if count == 0 or increment <= 0.0:
            return None, None

        center_index = (center - scan.angle_min) / increment
        half_index = half / increment
        first = int(math.ceil(center_index - half_index))
        last = int(math.floor(center_index + half_index))

        best = None
        best_angle = None
        for i in range(first, last + 1):
            index = i % count
            r = scan.ranges[index]
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                if best is None or r < best:
                    best = r
                    best_angle = scan.angle_min + index * increment
        return best, best_angle

    def _forward_clearance(self, scan):
        """Nearest obstacle inside the forward lane, plus the angle it was seen at.

        A plain angular sector is wrong here: the tray is only ~0.30 m wide, so
        even a 15 deg ray runs into the side wall at ~0.5 m and would be
        reported as an obstacle dead ahead. Hit points are therefore projected
        into the body frame and only those inside the robot's lane (|y| below
        `forward_lane_half_width`, x ahead) count as forward obstacles.
        """
        best = None
        best_angle = None
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                bx = r * math.cos(angle) + RADAR_X
                by = r * math.sin(angle) + RADAR_Y
                if bx > 0.0 and abs(by) <= self.lane_half:
                    if best is None or r < best:
                        best = r
                        best_angle = angle
            angle += scan.angle_increment
        return best, best_angle

    def tick(self):
        scan = self._scan
        if scan is not None:
            self.forward, self.forward_angle = self._forward_clearance(scan)
            self.left, self.left_angle = self._sector_min(
                scan, math.pi / 2.0, self.side_half)
            self.right, self.right_angle = self._sector_min(
                scan, -math.pi / 2.0, self.side_half)
            self.width = (self.left + self.right
                          if self.left is not None and self.right is not None
                          else None)

            if self.width is not None:
                self.width_sum += self.width
                self.width_count += 1
                self.width_min = (self.width if self.width_min is None
                                  else min(self.width_min, self.width))
                self.width_max = (self.width if self.width_max is None
                                  else max(self.width_max, self.width))
            if self.forward is not None:
                self.forward_min = (self.forward if self.forward_min is None
                                    else min(self.forward_min, self.forward))

        self._tick += 1
        if self._tick % self.sample_every == 0:
            self._record_sample()
        self.publish()

    def _record_sample(self):
        if len(self.samples) >= self.max_samples:
            del self.samples[:len(self.samples) - self.max_samples + 1]
        x, y, _, yaw = self._odom if self._odom else (None, None, None, None)
        self.samples.append({
            't': round(time.time(), 3),
            'x': x, 'y': y, 'yaw': yaw,
            'forward': self.forward,
            'left': self.left,
            'right': self.right,
            'width': self.width,
            'traveled': round(self.mileage, 4),
            'tray_length': self.tray_length(),
        })

    def tray_length(self):
        """Best tray-length estimate: measured map extent, else travelled path."""
        if self.map_length is not None:
            return self.map_length
        return self.mileage

    def _extract_xy(self, msg):
        """Pull planar (x, y) out of a PointCloud2, strided to max_map_points."""
        offsets = {}
        for field in msg.fields:
            if field.name in ('x', 'y') and field.datatype == PointField.FLOAT32:
                offsets[field.name] = field.offset
        if 'x' not in offsets or 'y' not in offsets:
            return []
        step = msg.point_step
        total = msg.width * msg.height
        if step <= 0 or total <= 0:
            return []
        stride = max(1, total // self.max_map_points)
        offset_x = offsets['x']
        offset_y = offsets['y']
        data = msg.data
        pts = []
        for i in range(0, total, stride):
            base = i * step
            try:
                x = struct.unpack_from('<f', data, base + offset_x)[0]
                y = struct.unpack_from('<f', data, base + offset_y)[0]
            except struct.error:
                break
            if math.isfinite(x) and math.isfinite(y):
                pts.append((x, y))
        return pts

    def measure_tray(self):
        """Measure the tray extent along the principal axis of the point cloud.

        A 2D PCA gives the dominant direction of the mapped tray; projecting
        every point onto it and taking min/max yields the measured length. On
        this course the corridor (~22 m along X) dominates the T-junction arms
        (~3.3 m along Y), so the principal axis follows the tray.
        """
        msg = self._map_msg
        if msg is None:
            return
        pts = self._extract_xy(msg)
        if len(pts) < 50:
            return

        count = len(pts)
        mean_x = sum(p[0] for p in pts) / count
        mean_y = sum(p[1] for p in pts) / count
        sxx = syy = sxy = 0.0
        for x, y in pts:
            dx = x - mean_x
            dy = y - mean_y
            sxx += dx * dx
            syy += dy * dy
            sxy += dx * dy
        sxx /= count
        syy /= count
        sxy /= count

        axis = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        ux = math.cos(axis)
        uy = math.sin(axis)

        lo = hi = None
        lo_pt = hi_pt = None
        for x, y in pts:
            proj = x * ux + y * uy
            if lo is None or proj < lo:
                lo, lo_pt = proj, (x, y)
            if hi is None or proj > hi:
                hi, hi_pt = proj, (x, y)

        self.map_axis = axis
        self.map_length = hi - lo
        self.tray_start, self.tray_end = lo_pt, hi_pt

    # ── publishing ───────────────────────────────────────────────────────────
    def status_text(self):
        source = 'map' if self.map_length is not None else 'odom'
        return (
            f'forward={_fmt(self.forward)} '
            f'left={_fmt(self.left)} right={_fmt(self.right)} '
            f'width={_fmt(self.width)} '
            f'traveled={_fmt(self.mileage)} '
            f'tray={_fmt(self.tray_length())}({source}) '
            f'samples={len(self.samples)}'
        )

    def publish(self):
        stamp = self.get_clock().now().to_msg()

        for pub, value in (
            (self.forward_pub, self.forward),
            (self.left_pub, self.left),
            (self.right_pub, self.right),
            (self.width_pub, self.width),
            (self.traveled_pub, self.mileage),
            (self.tray_pub, self.tray_length()),
        ):
            out = Float32()
            out.data = (float(value)
                        if value is not None and math.isfinite(value)
                        else float('nan'))
            pub.publish(out)

        text = String()
        text.data = self.status_text()
        self.status_pub.publish(text)

        self.marker_pub.publish(self._build_markers(stamp))

    def _color_for(self, distance):
        """Colour a measurement ray by how close the obstacle is."""
        if distance is None or not math.isfinite(distance):
            return (0.6, 0.6, 0.6)
        if distance <= self.near_distance:
            return (0.95, 0.2, 0.2)
        if distance <= self.warn_distance:
            return (0.95, 0.65, 0.1)
        return (0.2, 0.9, 0.3)

    def _build_markers(self, stamp):
        markers = MarkerArray()
        if self._odom is None:
            return markers

        x, y, z, yaw = self._odom
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        # LiDAR origin in odom (radar_link is unrotated w.r.t. base_link)
        radar_x = x + RADAR_X * cos_yaw - RADAR_Y * sin_yaw
        radar_y = y + RADAR_X * sin_yaw + RADAR_Y * cos_yaw

        def radar_to_odom(px, py):
            """radar-frame scan point -> odom."""
            gx = px + RADAR_X
            gy = py + RADAR_Y
            return (x + gx * cos_yaw - gy * sin_yaw,
                    y + gx * sin_yaw + gy * cos_yaw)

        # ── measurement rays (forward / left / right) ────────────────────────
        rays = Marker()
        rays.header.frame_id = 'odom'
        rays.header.stamp = stamp
        rays.ns = NS_RAYS
        rays.id = 0
        rays.type = Marker.LINE_LIST
        rays.action = Marker.ADD
        rays.scale.x = 0.012
        rays.lifetime = MARKER_LIFETIME
        rays.pose.orientation.w = 1.0
        for distance, angle in ((self.forward, self.forward_angle),
                                (self.left, self.left_angle),
                                (self.right, self.right_angle)):
            if distance is None or angle is None or not math.isfinite(distance):
                continue
            hx, hy = radar_to_odom(distance * math.cos(angle),
                                   distance * math.sin(angle))
            r, g, b = self._color_for(distance)
            rays.points.append(_point(radar_x, radar_y, z))
            rays.points.append(_point(hx, hy, z))
            rays.colors.append(_color(r, g, b, 1.0))
            rays.colors.append(_color(r, g, b, 1.0))
        if rays.points:
            markers.markers.append(rays)

        # ── clear-width span between the two walls ───────────────────────────
        if (self.left is not None and self.left_angle is not None
                and self.right is not None and self.right_angle is not None):
            lx, ly = radar_to_odom(self.left * math.cos(self.left_angle),
                                   self.left * math.sin(self.left_angle))
            rx, ry = radar_to_odom(self.right * math.cos(self.right_angle),
                                   self.right * math.sin(self.right_angle))
            span = Marker()
            span.header.frame_id = 'odom'
            span.header.stamp = stamp
            span.ns = NS_SPAN
            span.id = 0
            span.type = Marker.LINE_STRIP
            span.action = Marker.ADD
            span.scale.x = 0.02
            span.lifetime = MARKER_LIFETIME
            span.pose.orientation.w = 1.0
            span.points.append(_point(lx, ly, z))
            span.points.append(_point(rx, ry, z))
            span.colors.append(_color(0.3, 0.7, 1.0, 1.0))
            span.colors.append(_color(0.3, 0.7, 1.0, 1.0))
            markers.markers.append(span)

        # ── measured tray start / end ────────────────────────────────────────
        if self.tray_start is not None and self.tray_end is not None:
            ends = Marker()
            ends.header.frame_id = 'odom'
            ends.header.stamp = stamp
            ends.ns = NS_ENDS
            ends.id = 0
            ends.type = Marker.SPHERE_LIST
            ends.action = Marker.ADD
            ends.scale.x = ends.scale.y = ends.scale.z = 0.18
            ends.lifetime = MARKER_LIFETIME
            ends.pose.orientation.w = 1.0
            ends.points.append(_point(self.tray_start[0], self.tray_start[1], z))
            ends.points.append(_point(self.tray_end[0], self.tray_end[1], z))
            ends.colors.append(_color(1.0, 0.85, 0.1, 0.9))
            ends.colors.append(_color(1.0, 0.85, 0.1, 0.9))
            markers.markers.append(ends)

        # ── floating text readout ────────────────────────────────────────────
        source = 'map' if self.map_length is not None else 'odom'
        text = Marker()
        text.header.frame_id = 'odom'
        text.header.stamp = stamp
        text.ns = NS_TEXT
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = z + self.marker_z_offset
        text.pose.orientation.w = 1.0
        text.scale.z = 0.11
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.text = (
            f'里程 {self.mileage:.2f} m\n'
            f'前方 {_fmt(self.forward)}\n'
            f'左 {_fmt(self.left)}  右 {_fmt(self.right)}\n'
            f'净宽 {_fmt(self.width)}\n'
            f'桥架长 {_fmt(self.tray_length())} ({source})'
        )
        text.lifetime = MARKER_LIFETIME
        markers.markers.append(text)

        return markers

    # ── services ─────────────────────────────────────────────────────────────
    def status_cb(self, request, response):
        response.success = True
        response.message = self.status_text()
        self.get_logger().info(response.message)
        return response

    def reset_cb(self, request, response):
        self.mileage = 0.0
        self._last_xy = None
        self.width_sum = 0.0
        self.width_count = 0
        self.width_min = None
        self.width_max = None
        self.forward_min = None
        self.samples = []
        response.success = True
        response.message = 'Distance statistics and mileage reset'
        self.get_logger().info(response.message)
        return response

    def _report(self):
        width_avg = self.width_sum / self.width_count if self.width_count else None
        return {
            'generated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
            'frame': 'odom',
            'reference': 'radar_link (LiDAR frame)',
            'tray_length_m': self.tray_length(),
            'tray_length_source': 'map' if self.map_length is not None else 'odom',
            'tray_length_from_map_m': self.map_length,
            'tray_axis_deg': (math.degrees(self.map_axis)
                              if self.map_axis is not None else None),
            'tray_start': list(self.tray_start) if self.tray_start else None,
            'tray_end': list(self.tray_end) if self.tray_end else None,
            'traveled_m': self.mileage,
            'forward_m': self.forward,
            'forward_min_m': self.forward_min,
            'left_m': self.left,
            'right_m': self.right,
            'width_m': self.width,
            'width_min_m': self.width_min,
            'width_max_m': self.width_max,
            'width_avg_m': width_avg,
            'width_samples': self.width_count,
            'pose': ({'x': self._odom[0], 'y': self._odom[1], 'yaw': self._odom[3]}
                     if self._odom else None),
            'sample_count': len(self.samples),
        }

    def save_cb(self, request, response):
        if not self.report_dir_ok:
            response.success = False
            response.message = (
                f'Report directory {self.report_dir} is not writable')
            self.get_logger().error(response.message)
            return response
        report = self._report()
        stamp = time.strftime('%Y%m%d_%H%M%S')
        json_path = os.path.join(self.report_dir, f'distance_{stamp}.json')
        csv_path = os.path.join(self.report_dir, f'distance_{stamp}.csv')
        columns = ('t', 'x', 'y', 'yaw', 'forward', 'left', 'right', 'width',
                   'traveled', 'tray_length')
        try:
            with open(json_path, 'w') as handle:
                json.dump(report, handle, indent=2)
            with open(csv_path, 'w') as handle:
                handle.write(','.join(columns) + '\n')
                for sample in self.samples:
                    handle.write(','.join(
                        '' if sample[k] is None else f'{sample[k]:.6f}'
                        for k in columns) + '\n')
        except OSError as exc:
            response.success = False
            response.message = f'Save failed: {exc}'
            self.get_logger().error(response.message)
            return response
        response.success = True
        response.message = (
            f'Saved {len(self.samples)} samples | '
            f'tray={_fmt(report["tray_length_m"])} '
            f'traveled={_fmt(report["traveled_m"])} | {json_path}')
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = DistanceMeasure()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C and SIGTERM (e.g. from `timeout` or ros2 launch teardown)
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

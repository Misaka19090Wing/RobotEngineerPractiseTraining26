#!/usr/bin/env python3
"""
Waypoint Navigator for the PCD-based manual waypoint workflow.

Loads a saved 3D PCD map and publishes it for RViz context. Waypoints can be
added manually with the RViz 2D Goal Pose tool or with waypoint_cli.py. The node
then follows the waypoint queue with a differential-drive controller and uses
the live /scan topic for conservative obstacle avoidance.

Topics:
  /waypoint_map       (PointCloud2)      loaded PCD map
  /waypoint_markers   (MarkerArray)      waypoint queue
  /waypoint_path      (nav_msgs/Path)    remaining queue
  /goal_pose          (PoseStamped)      add a waypoint (RViz 2D Goal Pose)

Services:
  /waypoint/add     (rcl_interfaces/SetParameters)  add x y z yaw_deg
  /waypoint/start   (std_srvs/Trigger)
  /waypoint/stop    (std_srvs/Trigger)
  /waypoint/clear   (std_srvs/Trigger)
  /waypoint/save    (std_srvs/Trigger)
  /waypoint/load    (std_srvs/Trigger)
  /waypoint/status  (std_srvs/Trigger)
  /waypoint/reload_map (std_srvs/Trigger)
"""
import json
import math
import os
import struct
from collections import deque

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_MAP_DIR = os.path.expanduser('~/pointcloud_maps')
DEFAULT_WAYPOINT_FILE = os.path.expanduser('~/waypoint_files/waypoints.json')
RADAR_X = 0.063205
RADAR_Y = 0.015705


def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(value, low, high):
    return max(low, min(high, value))


def _yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _quaternion_from_yaw(yaw):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


def _newest_pcd(path):
    if os.path.isfile(path):
        return path
    if not os.path.isdir(path):
        return None
    pcds = [os.path.join(path, f) for f in os.listdir(path) if f.endswith('.pcd')]
    if not pcds:
        return None
    return max(pcds, key=os.path.getmtime)


def load_pcd(pcd_path, max_points=100000):
    """Load an ASCII or uncompressed binary PCD into (x, y, z, intensity)."""
    header = {}
    fields = []
    sizes = []
    types = []
    counts = []
    data_type = None

    with open(pcd_path, 'r', errors='replace') as f:
        while True:
            line = f.readline()
            if not line:
                break
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            parts = stripped.split()
            key = parts[0]
            if key == 'DATA':
                data_type = parts[1] if len(parts) > 1 else 'ascii'
                break
            if len(parts) > 1:
                header[key] = parts[1:]

        fields = header.get('FIELDS', [])
        sizes = [int(v) for v in header.get('SIZE', [])]
        types = header.get('TYPE', [])
        counts = [int(v) for v in header.get('COUNT', [])]
        if not sizes:
            sizes = [4] * len(fields)
        if not counts:
            counts = [1] * len(fields)

        width = int(header.get('WIDTH', ['0'])[0])
        height = int(header.get('HEIGHT', ['1'])[0])
        points = int(header.get('POINTS', ['0'])[0])
        point_count = points or width * height
        selected = [i for i, name in enumerate(fields) if name in ('x', 'y', 'z', 'intensity')]
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            raise ValueError('PCD file must contain x, y and z fields')

        step = max(1, point_count // max_points)
        result = []

        if data_type == 'ascii':
            n = 0
            for raw in f:
                values = raw.split()
                if len(values) < len(fields):
                    continue
                if n % step == 0:
                    row = []
                    for idx in selected:
                        try:
                            row.append(float(values[idx]))
                        except ValueError:
                            row.append(float('nan'))
                    if len(row) == 4:
                        result.append(tuple(row))
                    else:
                        result.append((row[0], row[1], row[2], 1.0))
                n += 1
        elif data_type in ('binary', 'binary_compressed'):
            if data_type == 'binary_compressed':
                raise ValueError('binary_compressed PCD is not supported')
            point_step = sum(s * c for s, c in zip(sizes, counts))
            fmt_map = {'F': 'f', 'I': 'i', 'U': 'I'}
            fmt = '<' + ''.join(fmt_map.get(t, 'f') * c for t, c in zip(types, counts))
            starts = []
            offset = 0
            for c in counts:
                starts.append(offset)
                offset += c
            data = f.read()
            n = 0
            for values in struct.iter_unpack(fmt, data[:point_count * point_step]):
                if n % step == 0:
                    row = []
                    for idx in selected:
                        row.append(float(values[starts[idx]]))
                    if len(row) == 4:
                        result.append(tuple(row))
                    else:
                        result.append((row[0], row[1], row[2], 1.0))
                n += 1
        else:
            raise ValueError(f'Unsupported PCD data type: {data_type}')

    return result


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('map_file', '')
        self.declare_parameter('map_dir', DEFAULT_MAP_DIR)
        self.declare_parameter('max_map_points', 100000)
        self.declare_parameter('waypoint_file', DEFAULT_WAYPOINT_FILE)
        self.declare_parameter('max_linear_speed', 0.8)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('linear_gain', 1.2)
        self.declare_parameter('angular_gain', 2.5)
        self.declare_parameter('goal_tolerance', 0.08)
        self.declare_parameter('yaw_tolerance', 0.18)
        self.declare_parameter('obstacle_stop_distance', 0.10)
        self.declare_parameter('obstacle_slow_distance', 0.35)
        self.declare_parameter('avoid_gain', 0.8)
        self.declare_parameter('control_rate_hz', 20.0)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.obstacle_stop = self.get_parameter('obstacle_stop_distance').value
        self.obstacle_slow = self.get_parameter('obstacle_slow_distance').value
        self.avoid_gain = self.get_parameter('avoid_gain').value
        self.waypoint_file = os.path.expanduser(
            self.get_parameter('waypoint_file').value)

        self.waypoints = deque()
        self.state = 'IDLE'
        self._odom = None
        self._scan = None
        self._last_odom_stamp = None
        self.map_points = []
        self.map_path = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_pub = self.create_publisher(
            PointCloud2, '/waypoint_map',
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.path_pub = self.create_publisher(Path, '/waypoint_path', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)

        self.start_srv = self.create_service(Trigger, '/waypoint/start', self.start_cb)
        self.stop_srv = self.create_service(Trigger, '/waypoint/stop', self.stop_cb)
        self.clear_srv = self.create_service(Trigger, '/waypoint/clear', self.clear_cb)
        self.save_srv = self.create_service(Trigger, '/waypoint/save', self.save_cb)
        self.load_srv = self.create_service(Trigger, '/waypoint/load', self.load_cb)
        self.status_srv = self.create_service(Trigger, '/waypoint/status', self.status_cb)
        self.reload_map_srv = self.create_service(
            Trigger, '/waypoint/reload_map', self.reload_map_cb)
        self.add_srv = self.create_service(SetParameters, '/waypoint/add', self.add_cb)
        self.add_on_set_parameters_callback(self._param_cb)

        rate = self.get_parameter('control_rate_hz').value
        self.tf_timer = self.create_timer(0.05, self._tf_pose_cb)
        self.control_timer = self.create_timer(1.0 / rate, self.control_loop)
        self.map_timer = self.create_timer(1.0, self.publish_map)

        self._load_pcd_map()
        os.makedirs(os.path.dirname(self.waypoint_file) or '.', exist_ok=True)

        self.get_logger().info(
            f'WaypointNavigator ready | state={self.state} '
            f'map={self.map_path or "none"} waypoints={len(self.waypoints)}')

    def _load_pcd_map(self):
        map_file = self.get_parameter('map_file').value
        map_dir = os.path.expanduser(self.get_parameter('map_dir').value)
        path = map_file or _newest_pcd(map_dir)
        if not path:
            self.get_logger().warn('No PCD map found; waypoint map display disabled')
            return
        max_points = self.get_parameter('max_map_points').value
        try:
            self.map_points = load_pcd(path, max_points=max_points)
            self.map_path = path
            self.get_logger().info(
                f'Loaded PCD map: {path} ({len(self.map_points)} displayed points)')
        except Exception as exc:
            self.get_logger().error(f'Failed to load PCD map {path}: {exc}')

    def publish_map(self):
        if not self.map_points:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'odom'
        points = self.map_points
        pc = PointCloud2()
        pc.header = header
        pc.height = 1
        pc.width = len(points)
        pc.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pc.is_bigendian = False
        pc.point_step = 16
        pc.row_step = 16 * len(points)
        pc.is_dense = True
        pc.data = b''.join(
            struct.pack('<ffff', x, y, z, i) for x, y, z, i in points)
        self.map_pub.publish(pc)

    def odom_cb(self, msg):
        p = msg.pose.pose
        self._odom = (
            p.position.x,
            p.position.y,
            p.position.z,
            _yaw_from_quaternion(p.orientation),
        )
        self._last_odom_stamp = self.get_clock().now()

    def scan_cb(self, msg):
        self._scan = msg

    def _tf_pose_cb(self):
        for frame in ('base_footprint', 'base_link'):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'odom', frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05))
            except TransformException:
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            self._odom = (
                t.x,
                t.y,
                t.z,
                _yaw_from_quaternion(q),
            )
            self._last_odom_stamp = self.get_clock().now()
            return

    def _to_odom(self, pose_stamped):
        frame = pose_stamped.header.frame_id
        if not frame or frame == 'odom':
            return pose_stamped.pose
        try:
            transformed = self.tf_buffer.transform(
                pose_stamped, 'odom',
                timeout=rclpy.duration.Duration(seconds=0.3))
            return transformed.pose
        except TransformException as exc:
            self.get_logger().warn(
                f'Could not transform goal from {frame} to odom: {exc}')
            return None

    def goal_cb(self, msg):
        pose = self._to_odom(msg)
        if pose is None:
            return
        wp = {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'yaw': _yaw_from_quaternion(pose.orientation),
        }
        self.waypoints.append(wp)
        self.get_logger().info(
            f'Waypoint added [{len(self.waypoints)}]: '
            f'({wp["x"]:.2f}, {wp["y"]:.2f}, {wp["z"]:.2f}, yaw={math.degrees(wp["yaw"]):.1f}deg)')
        self._publish_visuals()

    def add_cb(self, request, response):
        for param in request.parameters:
            if param.name == 'goal':
                values = param.value.double_array_value
                if len(values) >= 4:
                    wp = {
                        'x': values[0],
                        'y': values[1],
                        'z': values[2],
                        'yaw': math.radians(values[3]),
                    }
                    self.waypoints.append(wp)
                    self.get_logger().info(
                        f'Waypoint added via service [{len(self.waypoints)}]: '
                        f'({wp["x"]:.2f}, {wp["y"]:.2f}, {wp["z"]:.2f}, '
                        f'yaw={values[3]:.1f}deg)')
                    self._publish_visuals()
                    response.results.append(
                        SetParametersResult(successful=True, reason='waypoint added'))
                    return response
        response.results.append(
            SetParametersResult(
                successful=False,
                reason='goal parameter must be a double array of x y z yaw_deg'))
        return response

    def start_cb(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints in queue'
            return response
        self.state = 'NAVIGATING'
        response.success = True
        response.message = f'Navigating to {len(self.waypoints)} waypoint(s)'
        self.get_logger().info(response.message)
        self._publish_visuals()
        return response

    def stop_cb(self, request, response):
        self.state = 'IDLE'
        self.cmd_pub.publish(Twist())
        response.success = True
        response.message = 'Waypoint navigation stopped'
        self.get_logger().info(response.message)
        return response

    def clear_cb(self, request, response):
        self.state = 'IDLE'
        self.cmd_pub.publish(Twist())
        self.waypoints.clear()
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))
        self._publish_visuals()
        response.success = True
        response.message = 'Waypoint queue cleared'
        self.get_logger().info(response.message)
        return response

    def save_cb(self, request, response):
        try:
            data = {
                'state': self.state,
                'waypoints': list(self.waypoints),
                'map': self.map_path,
            }
            with open(self.waypoint_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
            response.success = True
            response.message = f'Saved {len(self.waypoints)} waypoints to {self.waypoint_file}'
        except Exception as exc:
            response.success = False
            response.message = f'Save failed: {exc}'
        self.get_logger().info(response.message)
        return response

    def load_cb(self, request, response):
        if not os.path.isfile(self.waypoint_file):
            response.success = False
            response.message = f'Waypoint file not found: {self.waypoint_file}'
            return response
        try:
            with open(self.waypoint_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            self.waypoints = deque(data.get('waypoints', []))
            self.state = 'IDLE'
            self.cmd_pub.publish(Twist())
            self._publish_visuals()
            response.success = True
            response.message = f'Loaded {len(self.waypoints)} waypoints'
        except Exception as exc:
            response.success = False
            response.message = f'Load failed: {exc}'
        self.get_logger().info(response.message)
        return response

    def status_cb(self, request, response):
        current = ''
        if self.waypoints:
            wp = self.waypoints[0]
            current = f'({wp["x"]:.2f}, {wp["y"]:.2f}, {wp["z"]:.2f})'
        odom = ''
        if self._odom:
            odom = f'({self._odom[0]:.2f}, {self._odom[1]:.2f}, {self._odom[2]:.2f})'
        response.success = True
        response.message = (
            f'state={self.state} queued={len(self.waypoints)} '
            f'target={current or "none"} odom={odom or "none"} '
            f'map={self.map_path or "none"}')
        self.get_logger().info(response.message)
        return response

    def reload_map_cb(self, request, response):
        map_file = self.get_parameter('map_file').value
        map_dir = os.path.expanduser(self.get_parameter('map_dir').value)
        path = map_file or _newest_pcd(map_dir)
        if not path:
            response.success = False
            response.message = 'No PCD map found'
            return response
        max_points = self.get_parameter('max_map_points').value
        try:
            self.map_points = load_pcd(path, max_points=max_points)
            self.map_path = path
            self.publish_map()
            response.success = True
            response.message = (
                f'Reloaded PCD map: {path} ({len(self.map_points)} points)')
        except Exception as exc:
            response.success = False
            response.message = f'Reload failed: {exc}'
        self.get_logger().info(response.message)
        return response

    def _param_cb(self, params):
        for param in params:
            name = param.name
            if name == 'max_linear_speed':
                self.max_linear_speed = param.value
            elif name == 'max_angular_speed':
                self.max_angular_speed = param.value
            elif name == 'linear_gain':
                self.linear_gain = param.value
            elif name == 'angular_gain':
                self.angular_gain = param.value
            elif name == 'goal_tolerance':
                self.goal_tolerance = param.value
            elif name == 'yaw_tolerance':
                self.yaw_tolerance = param.value
            elif name == 'obstacle_stop_distance':
                self.obstacle_stop = param.value
            elif name == 'obstacle_slow_distance':
                self.obstacle_slow = param.value
            elif name == 'avoid_gain':
                self.avoid_gain = param.value
        return SetParametersResult(successful=True, reason='speed parameters updated')

    def _publish_visuals(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        if not self.waypoints:
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.action = Marker.DELETEALL
            self.marker_pub.publish(MarkerArray(markers=[marker]))
        else:
            self._fill_marker_array(markers, stamp)
            self.marker_pub.publish(markers)

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = 'odom'
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.position.z = wp['z']
            qx, qy, qz, qw = _quaternion_from_yaw(wp['yaw'])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)
        self.path_pub.publish(path)

    def _fill_marker_array(self, markers, stamp):
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'odom'
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = wp['z']
            qx, qy, qz, qw = _quaternion_from_yaw(wp['yaw'])
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = 0.28
            marker.scale.y = 0.04
            marker.scale.z = 0.04
            marker.color.a = 1.0
            marker.color.r = 0.25
            marker.color.g = 0.9
            marker.color.b = 0.2
            marker.lifetime = Duration()
            if i == 0 and self.state == 'NAVIGATING':
                marker.color.r = 1.0
                marker.color.g = 0.6
                marker.color.b = 0.1
            markers.markers.append(marker)

    def _scan_avoidance(self):
        scan = self._scan
        if scan is None:
            return math.inf, 0.0
        angle = scan.angle_min
        left_risk = 0.0
        right_risk = 0.0
        min_front = math.inf
        for r in scan.ranges:
            if not math.isnan(r) and scan.range_min < r < scan.range_max:
                if r < 0.8:
                    px = r * math.cos(angle)
                    py = r * math.sin(angle)
                    bx = px + RADAR_X
                    by = py + RADAR_Y
                    bearing = math.atan2(by, bx)
                    if bx > 0.02 and abs(bearing) < math.pi / 2.0:
                        distance = math.hypot(bx, by)
                        min_front = min(min_front, distance)
                        if distance < 0.6:
                            weight = max(0.0, 1.0 - distance / 0.6)
                            if by > 0.0:
                                left_risk += weight
                            else:
                                right_risk += weight
            angle += scan.angle_increment

        avoid = 0.0
        if min_front < 0.6:
            total = left_risk + right_risk
            if total > 1e-6:
                avoid = self.avoid_gain * (right_risk - left_risk) / total
        return min_front, avoid

    def control_loop(self):
        if self.state != 'NAVIGATING':
            return
        if self._last_odom_stamp is not None:
            age = (self.get_clock().now() - self._last_odom_stamp).nanoseconds * 1e-9
            if age > 1.0:
                self.get_logger().warn('Odom timeout, stopping navigation')
                self.stop_cb(None, Trigger.Response())
                return
        if self._odom is None:
            self.cmd_pub.publish(Twist())
            return
        if not self.waypoints:
            self.state = 'IDLE'
            self.cmd_pub.publish(Twist())
            return

        x, y, _z, yaw = self._odom
        target = self.waypoints[0]
        dx = target['x'] - x
        dy = target['y'] - y
        distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_error = _normalize_angle(desired_yaw - yaw)

        twist = Twist()
        if distance <= self.goal_tolerance:
            if abs(yaw_error) <= self.yaw_tolerance:
                self.waypoints.popleft()
                self.get_logger().info(
                    f'Reached waypoint, {len(self.waypoints)} remaining')
                self._publish_visuals()
                if not self.waypoints:
                    self.state = 'IDLE'
                    self.cmd_pub.publish(Twist())
                return
            twist.linear.x = 0.0
            twist.angular.z = _clamp(
                yaw_error * self.angular_gain, -self.max_angular_speed, self.max_angular_speed)
        else:
            linear = _clamp(distance * self.linear_gain, 0.0, self.max_linear_speed)
            angular = _clamp(
                yaw_error * self.angular_gain, -self.max_angular_speed, self.max_angular_speed)
            if abs(yaw_error) > math.radians(75.0):
                linear = 0.0
            elif abs(yaw_error) > math.radians(30.0):
                linear = min(linear, 0.06)

            front, avoid = self._scan_avoidance()
            if front < self.obstacle_stop:
                linear = 0.0
            elif front < self.obstacle_slow:
                linear *= max(
                    0.0, (front - self.obstacle_stop)
                    / (self.obstacle_slow - self.obstacle_stop))
            angular = _clamp(
                angular + avoid, -self.max_angular_speed, self.max_angular_speed)
            twist.linear.x = linear
            twist.angular.z = angular

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

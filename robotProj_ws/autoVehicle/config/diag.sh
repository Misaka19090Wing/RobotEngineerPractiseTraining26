#!/bin/bash
# Point cloud mapping diagnostic — run while the system is running
echo "========================================="
echo "  Point Cloud Mapping Diagnosis"
echo "========================================="
echo ""

echo "--- ROS topics (scan/lidar/odom) ---"
ros2 topic list 2>/dev/null | grep -E "scan|lidar|odom|pointcloud" || echo "(none found)"
echo ""

echo "--- /scan info ---"
ros2 topic info /scan 2>/dev/null || echo "  /scan does NOT exist"
echo ""

echo "--- /lidar_points info ---"
ros2 topic info /lidar_points 2>/dev/null || echo "  /lidar_points does NOT exist"
echo ""

echo "--- /odom info ---"
ros2 topic info /odom 2>/dev/null || echo "  /odom does NOT exist"
echo ""

echo "--- /pointcloud_map info ---"
ros2 topic info /pointcloud_map 2>/dev/null || echo "  /pointcloud_map does NOT exist"
echo ""

echo "--- Gazebo LiDAR topic type ---"
gz topic -i -t /model/autoVehicle/link/radar_link/sensor/lidar/scan 2>/dev/null | head -5 || echo "  cannot get Gazebo topic info"
echo ""

echo "--- Gazebo PointCloud topic type ---"
gz topic -i -t /model/autoVehicle/link/radar_link/sensor/lidar/scan/points 2>/dev/null | head -5 || echo "  cannot get Gazebo topic info"
echo ""

echo "--- Gazebo odometry topic type ---"
gz topic -i -t /model/autoVehicle/odometry 2>/dev/null | head -5 || echo "  cannot get Gazebo topic info"
echo ""

echo "--- TF frames ---"
ros2 run tf2_tools view_frames 2>/dev/null
echo "Check /tmp/frames.pdf for TF tree"
echo ""

echo "--- Bridge nodes ---"
ros2 node list 2>/dev/null | grep bridge || echo "(no bridge nodes found)"
echo ""

echo "Diagnostic complete. Send the output above."

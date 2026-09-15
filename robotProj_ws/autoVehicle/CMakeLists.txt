cmake_minimum_required(VERSION 3.8)
project(autoVehicle)

# find dependencies
find_package(ament_cmake REQUIRED)

# install directories
install(DIRECTORY config launch maps meshes urdf world scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
install(PROGRAMS
  scripts/diff_drive_controller.py
  scripts/pointcloud_mapper.py
  scripts/synthetic_lidar.py
  scripts/pose_tf_broadcaster.py
  scripts/waypoint_navigator.py
  scripts/waypoint_cli.py
  scripts/distance_measure.py
  scripts/distance_cli.py
  scripts/generate_course_map.py
  DESTINATION lib/${PROJECT_NAME}
)

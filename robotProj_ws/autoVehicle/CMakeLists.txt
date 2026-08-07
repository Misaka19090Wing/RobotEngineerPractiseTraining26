cmake_minimum_required(VERSION 3.8)
project(autoVehicle)

# find dependencies
find_package(ament_cmake REQUIRED)

# install directories
install(DIRECTORY config launch meshes urdf world scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
install(PROGRAMS
  scripts/diff_drive_controller.py
  scripts/pointcloud_mapper.py
  scripts/pose_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

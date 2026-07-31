cmake_minimum_required(VERSION 3.8)
project(autoVehicle)

# find dependencies
find_package(ament_cmake REQUIRED)

# install directories
install(DIRECTORY config launch meshes urdf world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

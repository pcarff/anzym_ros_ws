cmake_minimum_required(VERSION 3.5)
project(anzym_description)

# Find dependencies (ROS 2)
find_package(ament_cmake REQUIRED)

# Install directories (URDF, Meshes, Launch)
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_package()

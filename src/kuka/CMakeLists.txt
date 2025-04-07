cmake_minimum_required(VERSION 3.5)
project(kuka)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)  # Add this for ROS 2 compatibility
find_package(robot_state_publisher REQUIRED)  # Ensures dependency is found

# Install necessary files
install(DIRECTORY share urdf meshes launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

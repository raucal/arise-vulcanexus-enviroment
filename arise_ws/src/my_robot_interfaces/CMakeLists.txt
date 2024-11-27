cmake_minimum_required(VERSION 3.5)
project(my_robot_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Locate and configure packages required by the project.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

# Define a CMake variable named dependencies that lists all
# ROS 2 packages and other dependencies the project requires.
set(dependencies
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetCoordinates.msg"
  "msg/Fruits.msg"
  "msg/JXCP1Inputs.msg"
  "srv/CheckInt.srv"
  "srv/List.srv"
  "srv/PickObject.srv"
  "srv/FruitInfo.srv"
  "srv/CheckGeometry.srv"
  "srv/TransformCoordinates.srv"
  DEPENDENCIES std_msgs geometry_msgs 
)



# ament_target_dependencies(executable_name rclcpp)
# ament_target_dependencies(executable_name rclpy)
# ament_target_dependencies(executable_name geometry_msgs)

# ament_export_dependencies(geometry_msgs)
# ament_export_dependencies(sensor_msgs)
# ament_export_dependencies(realsense2_camera_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(kaia_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(twist_to_rpm src/twist_to_rpm.cpp)
ament_target_dependencies(twist_to_rpm rclcpp geometry_msgs std_msgs)

install(TARGETS twist_to_rpm DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch urdf config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/odom_bridge.py DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(test_slam_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/odom_tf_bridge.py scripts/cmd_vel_bridge.py scripts/motor_odom.py scripts/awsd_teleop.py scripts/imu_remap.py scripts/map_room.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

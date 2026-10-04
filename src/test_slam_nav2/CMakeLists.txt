cmake_minimum_required(VERSION 3.8)
project(test_slam_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/odom_tf_bridge.py scripts/cmd_vel_bridge.py scripts/motor_odom.py scripts/awsd_teleop.py scripts/imu_remap.py scripts/map_room.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  file(GLOB PYTHON_TEST_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.py"
  )
  foreach(TEST_FILE IN LISTS PYTHON_TEST_FILES)
    get_filename_component(TEST_NAME "${TEST_FILE}" NAME_WE)
    ament_add_pytest_test("${TEST_NAME}" "${TEST_FILE}")
  endforeach()
endif()

ament_package()

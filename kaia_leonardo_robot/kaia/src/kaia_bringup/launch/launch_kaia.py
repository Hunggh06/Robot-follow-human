#!/usr/bin/env python3
"""launch_kaia — mirror launch_leonardo.py nhưng cho firmware Kaia single-ESP32.

Chuỗi: joy -> teleop_twist_joy (/cmd_vel_joy) -> twist_mux (/cmd_vel_mux)
  -> twist_to_rpm (/motor/*_target_rpm) -> ESP32
  ESP32 -> /telemetry -> kaiaai_telemetry (/scan) + /wheel/odom -> odom_bridge -> TF.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    bringup_share = get_package_share_directory('kaia_bringup')
    mux_yaml = os.path.join(bringup_share, 'config', 'config_param_cmd_vel_mux.yaml')

    # test_slam_nav2 của bạn đã chạy cmd_vel_bridge + odom_tf_bridge.
    # Chạy launch này song song thì tắt 2 node trùng để khỏi double-publish:
    #   ros2 launch kaia_bringup launch_kaia.py use_twist_to_rpm:=false use_odom_bridge:=false
    ld.add_action(DeclareLaunchArgument('use_twist_to_rpm', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_odom_bridge', default_value='true'))

    ld.add_action(Node(package='joy', executable='joy_node', name='game_controller',
        parameters=[{'device_id': 1, 'deadzone': 0.05, 'autorepeat_rate': 0.0,
                     'sticky_buttons': False, 'coalesce_interval_ms': 1}],
        output='screen'))
    ld.add_action(Node(package='teleop_twist_joy', executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[{'require_enable_button': True, 'enable_button': 5,
                     'enable_turbo_button': -1, 'axis_linear.x': 1,
                     'scale_linear.x': 0.36, 'axis_angular.yaw': 2,
                     'scale_angular.yaw': 1.0, 'publish_stamped_twist': False}],
        remappings=[('cmd_vel', '/cmd_vel_joy')], output='screen'))
    ld.add_action(Node(package='twist_mux', executable='twist_mux', name='twist_mux',
        parameters=[mux_yaml],
        remappings=[('cmd_vel_out', '/cmd_vel_mux')],
        output='screen'))
    ld.add_action(Node(package='kaia_bringup', executable='twist_to_rpm',
        name='twist_to_rpm', output='screen',
        condition=IfCondition(LaunchConfiguration('use_twist_to_rpm'))))
    ld.add_action(Node(package='kaia_bringup', executable='odom_bridge.py',
        name='odom_bridge', output='screen',
        condition=IfCondition(LaunchConfiguration('use_odom_bridge'))))
    # Kaia telemetry decoder (gói sẵn có của bạn trong ros2_ws/src)
    ld.add_action(Node(package='kaiaai_telemetry', executable='telem',
        name='kaiaai_telemetry_node', output='screen',
        parameters=[{'laser_scan.lidar_model': 'XIAOMI-LDS02RR',
                     'laser_scan.frame_id': 'laser_frame',
                     'telemetry.topic_name_sub': 'telemetry',
                     'laser_scan.topic_name_pub': 'scan'}]))
    ld.add_action(Node(package='tf2_ros', executable='static_transform_publisher',
        name='base_to_laser', output='log',
        arguments=['0', '0', '0.10', '0', '0', '0', 'base_footprint', 'laser_frame']))
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888'],
        output='screen'))
    with open(os.path.join(bringup_share, 'urdf', 'kaia.urdf')) as f:
        urdf_xml = f.read()
    ld.add_action(Node(package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_xml}],
        output='screen'))
    return ld

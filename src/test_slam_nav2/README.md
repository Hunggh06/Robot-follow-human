# test_slam_nav2 — Test SLAM + Nav2 cho xebamnguoi (ESP32 low-level)

URDF copy tu `xebamnguoi` (goi ROS1 SolidWorks export) sang package ROS2 Humble,
giu nguyen meshes + vi tri khop. Dieu khien thap tang la ESP32 micro-ROS:
`/cmd_vel` (sub), `/wheel/odom` + `/scan` (pub).

## Cay TF
```
map -> odom          (slam_toolbox / amcl)
odom -> base_link    (odom_tf_bridge.py, tu /wheel/odom ESP32)
base_link -> lidar_link (urdf) -> laser (static, trung nhau)
```

## Build
```bash
cd /home/hungdo/ros2_ws
colcon build --packages-select test_slam_nav2
source install/setup.bash
```

## 1. Xem URDF (khong can robot)
```bash
ros2 launch test_slam_nav2 display.launch.py
```

## 2. Test SLAM that (can ESP32 + agent)
```bash
# terminal 1: agent (doi ESP32 ket noi)
docker run --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
# terminal 2:
ros2 launch test_slam_nav2 slam.launch.py
# day robot di quanh phong de ve map, roi luu:
ros2 run nav2_map_server map_saver_cli -f ~/maps/phong1
```

## 3. Test Nav2 (can map)
```bash
ros2 launch test_slam_nav2 navigation.launch.py map:=/home/hungdo/maps/phong1.yaml
# Trong RViz: 2D Pose Estimate -> dat vi tri -> 2D Goal Pose -> robot tu chay
```

## Luu y
- Khoang cach 2 banh URDF = 0.2385m, firmware ESP32 dang 0.241m (lech 2.5mm,
  khong dang ke; muon khop thi sua WHEEL_BASE trong .ino roi nap lai).
- `/scan` frame `laser` dang gan cung goc `lidar_link`. Ve lau dai nen doi
  firmware pub frame `lidar_link` luon va bo static TF.
- Toc do Nav2 gioi han 0.4 m/s de PID ESP32 (RPM_MAX 200) theo kip.

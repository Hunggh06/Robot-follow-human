# Kaia Leonardo Robot — khung Leonardo + firmware Kaia

Bê cấu trúc từ [EmDonato/Esp32_micro_Ros_mobile_robot_Leonardo](https://github.com/EmDonato/Esp32_micro_Ros_mobile_robot_Leonardo),
nhưng chạy firmware Kaia single-ESP32 của bạn (LDS02RR raw → `/telemetry` → `kaiaai_telemetry` → `/scan`).

```
kaia_leonardo_robot/
├── kaia/                    # Companion PC (mirror Leonardo/) — ROS 2 Humble
│   ├── config/              # cmd_vel_mux, nav2, slam params
│   └── src/
│       ├── kaia_bringup/    # mirror leonardo pkg: launch + urdf + twist_to_rpm
│       ├── odom_to_tf_cpp/  # copy nguyên từ Leonardo (tham khảo)
│       ├── laser_tf_cpp/    # copy nguyên từ Leonardo (tham khảo)
│       └── cmd_vel_normalizer/ # copy nguyên từ Leonardo
├── kaia_esp32/              # Firmware (mirror Leonardo_esp32/ modular)
│   ├── kaia_esp32.ino       # setup()/loop() gọn
│   ├── config.h             # MỌI chân + hằng số (TPR=234 KHÔNG ĐỔI)
│   ├── encoder.h/.cpp       # class Encoder kiểu Leonardo, ruột ISR của bạn
│   ├── motor_control.h/.cpp # MotorDriver L298N + PID 10Hz (phần được phép sửa)
│   ├── lidar_kaia.h         # lidarPoll() NGUYÊN BẢN — CẤM SỬA LOGIC
│   ├── odometry_kaia.h      # updateOdometry() nguyên bản
│   └── micro_ros_kaia_fun.h # transport Kaia 2 tham số + sync time
├── libraries_arduinoIDE/    # copy nguyên MotorDriver + PID từ Leonardo
├── chassis_kaia_3Dprint/    # placeholder (Leonardo có STL/F3D)
└── docs/
```

## Khác biệt Kaia vs Leonardo (bắt buộc nhớ)

| Leonardo gốc | Kaia của bạn |
|---|---|
| 2 ESP32 (main + lidar LD06) | 1 ESP32 (motor + LDS02RR raw chung) |
| `set_microros_wifi_transports(ssid,pass,ip,port)` 4 tham số | `set_microros_wifi_transports(ip,port)` **2 tham số** (`micro_ros_kaia.h`), gọi 4 tham số = Guru Meditation |
| `/cmd_vel_mux` (Twist) trực tiếp vào ESP32 | ESP32 nghe `/motor/left_target_rpm` + `/motor/right_target_rpm` (Float32) → cần node `twist_to_rpm` trên PC |
| odom có `frame_id` đầy đủ | odom ESP32 để trống `frame_id` → `odom_bridge.py` tự điền `odom`/`base_footprint` |
| time_sync topic | `rmw_uros_sync_session()` lấy giờ host |
| agent `udp6` | agent `udp4 --port 8888` (IP `192.168.1.22`) |
| TPR 994, base 0.20 | **TPR 234, base 0.2385 (đo thước), wheel 0.065 — KHÔNG ĐỔI** |

## Mang sang `ros2_ws/src` khi sẵn sàng

Thư mục này đứng NGOÀI `src/` để không làm bẩn colcon. Khi muốn build:

```bash
cp -r ~/ros2_ws/kaia_leonardo_robot/kaia/src/kaia_bringup ~/ros2_ws/src/
# odom_to_tf_cpp / laser_tf_cpp / cmd_vel_normalizer chỉ copy nếu chưa có
colcon build --packages-select kaia_bringup
source install/setup.bash
ros2 launch kaia_bringup launch_kaia.py
```

`kaiaai_msgs` + `kaiaai_telemetry` của bạn vẫn nằm ở `src/`, launch Kaia tự tìm và dùng tiếp —
không duplicate.

## Firmware: phần nào được sửa / cấm sửa

- **CẤM SỬA LOGIC:** `lidar_kaia.h::lidarPoll()` — copy nguyên bản: `LDS_BUF_MAX 400`,
  `PUB_PERIOD 30ms`, PWM lidar `10kHz/11bit/1023`, `setRxBufferSize(4096)` trước `begin()`,
  không checksum/header, stamp từ `rmw_uros_epoch_nanos()`, best-effort publish.
- **ĐƯỢC PHÉP SỬA (đã sửa):** `motor_control.*` — refactor PID inline của bạn sang class
  `PID` kiểu Leonardo (vẫn Kp 1.8 / Ki 2.5 / Kd 0.15, integral ±300, lọc RPM alpha 0.25,
  timeout 500ms, feedforward TẮT để giữ đúng tay tune của bạn), + `encoder.*` dùng class
  `Encoder` với đúng luật dấu ISR của bạn (trái `A==B ++`, phải `A==B --`), `RISING` chân A.

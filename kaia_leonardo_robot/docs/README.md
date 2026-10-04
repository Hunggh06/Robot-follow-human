# Docs — đối chiếu Leonardo ↔ Kaia

- `https://github.com/EmDonato/Esp32_micro_Ros_mobile_robot_Leonardo`
- Ba hằng số đã đo bằng DO (giữ nguyên trong `kaia_esp32/config.h`):
  `LDS_BUF_MAX 400`, `PUB_PERIOD_MS 30`, lidar PWM `10k/11b/1023`.
- WiFi 2.4GHz: `LK 01.29` (bỏ `- 5Ghz`). Agent `192.168.1.22:8888` udp4.
- Nạp firmware: mở `kaia_esp32/kaia_esp32.ino` bằng Arduino IDE,
  cài `micro_ros_arduino_kaia`, chọn board ESP32 DevKit, baud 115200.

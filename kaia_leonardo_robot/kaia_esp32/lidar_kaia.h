// lidar_kaia.h — NGUYÊN BẢN từ code của bạn. CẤM SỬA LOGIC.
// Chỉ tách ra header để .ino gọn kiểu Leonardo. Mọi hằng số giữ nguyên:
// LDS_BUF_MAX 400 (567B > XRCE 512), PUB_PERIOD 30ms (8.0kB/s < 14.85kB/s),
// PWM 10kHz/11bit/1023, setRxBufferSize(4096) TRƯỚC begin().
// KHÔNG checksum, KHÔNG tìm header, KHÔNG cắt 22 byte — node PC tự sync 0xFA.
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <rmw_microros/rmw_microros.h>
#include <kaiaai_msgs/msg/kaiaai_telemetry2.h>
#include "config.h"

extern HardwareSerial LIDAR;
extern uint8_t ldsBuf[LDS_BUF_MAX];
extern uint32_t telemSeq;
extern rcl_publisher_t telem_pub;
extern kaiaai_msgs__msg__KaiaaiTelemetry2 telem_msg;

inline void lidarInit() {
  ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_RES);
  ledcAttachPin(LIDAR_MOT_EN, LIDAR_PWM_CHANNEL);
  ledcWrite(LIDAR_PWM_CHANNEL, LIDAR_PWM);
  LIDAR.setRxBufferSize(4096);
  LIDAR.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
}

inline void lidarPoll() {
  static uint32_t lastPub = 0;
  static uint32_t lastLog = 0;
  static uint32_t bytesInWindow = 0;

  uint32_t n = 0;
  while (LIDAR.available() > 0 && n < LDS_BUF_MAX) {
    ldsBuf[n++] = (uint8_t)LIDAR.read();
  }
  bytesInWindow += n;

  if (millis() - lastLog >= 2000) {
    Serial.print("[LDS] ");
    Serial.print(bytesInWindow);
    Serial.print(" byte/2s = ");
    Serial.print(bytesInWindow * 1000 / 2000);
    Serial.println(" B/s");
    bytesInWindow = 0;
    lastLog = millis();
  }

  if (n == 0) return;
  if (millis() - lastPub < PUB_PERIOD_MS) return;

  memset(&telem_msg, 0, sizeof(telem_msg));

  int64_t ns = rmw_uros_epoch_nanos();
  if (ns > 0) {
    telem_msg.stamp.sec     = (uint32_t)(ns / 1000000000LL);
    telem_msg.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  }

  telem_msg.seq             = ++telemSeq;
  telem_msg.lds.data        = ldsBuf;
  telem_msg.lds.size        = n;
  telem_msg.lds.capacity    = LDS_BUF_MAX;
  telem_msg.wifi_rssi_dbm   = (int8_t)WiFi.RSSI();
  telem_msg.battery_mv      = 0;
  telem_msg.scan_start_hint = false;

  rcl_publish(&telem_pub, &telem_msg, NULL);
  lastPub = millis();
}

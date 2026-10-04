// config.h — MỌI hằng số Kaia ở một chỗ (mirror Variable.hpp của Leonardo).
// TPR=234, WHEEL_BASE=0.2385, WHEEL_DIAMETER=0.065 là SỐ ĐO THẬT — KHÔNG ĐỔI.
#pragma once
#include <Arduino.h>

// ---- WiFi / Agent (Kaia fork: 2 tham số!) ----
#define WIFI_SSID     "LK 01.29"
#define WIFI_PASSWORD "12312345"
#define AGENT_IP      "192.168.1.22"
#define AGENT_PORT    8888

// ---- MOTOR PIN (giữ nguyên bản của bạn) ----
#define IN1 22
#define IN2 23
#define ENA 32
#define IN3 2
#define IN4 4
#define ENB 33

// ---- ENCODER PIN (giữ nguyên) ----
#define ENC_L_A 27
#define ENC_L_B 14
#define ENC_R_A 26
#define ENC_R_B 25

// ---- LIDAR (CẤM ĐỔI — số của Kaia) ----
#define LIDAR_RX 17
#define LIDAR_TX 16
#define LIDAR_MOT_EN 15
#define LIDAR_BAUD 115200
#define LIDAR_PWM_CHANNEL 2
#define LIDAR_PWM_FREQ    10000
#define LIDAR_PWM_RES     11
#define LIDAR_PWM         1023

// ---- MOTOR PWM ----
#define PWM_FREQ    1000
#define PWM_RES     8
#define PWM_CH_LEFT  0
#define PWM_CH_RIGHT 1
#define MAX_PWM 255

// ---- ROBOT (SỐ ĐO THẬT) ----
#define ENCODER_TPR 234.0f
#define WHEEL_DIAMETER 0.065f
#define WHEEL_RADIUS    (WHEEL_DIAMETER / 2.0f)
#define WHEEL_CIRCUMFERENCE (3.14159265358979323846f * WHEEL_DIAMETER)
#define WHEEL_BASE 0.2385f

// ---- PID (tay tune của bạn, giữ nguyên) ----
#define PID_PERIOD_MS 100
#define INTEGRAL_LIMIT 300.0f
#define CMD_TIMEOUT_MS 500
#define RPM_FILTER_ALPHA 0.25f

// ---- TELEMETRY (CẤM ĐỔI — đã đo bằng DO) ----
#define LDS_BUF_MAX 400
#define PUB_PERIOD_MS 30

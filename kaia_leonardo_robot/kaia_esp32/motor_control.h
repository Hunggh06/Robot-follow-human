// motor_control.h — phần ĐỘNG CƠ được refactor theo kiểu Leonardo
// (MotorDriver L298N + class PID có feedforward), nhưng giữ đúng số của bạn:
// pins IN1/IN2/ENA..., Kp 1.8/Ki 2.5/Kd 0.15, MAX_PWM 255, PID 10Hz,
// lọc RPM alpha 0.25, timeout 500ms. Feedforward TẮT để giữ tay tune của bạn.
#pragma once
#include <Arduino.h>

void motorInit();
void motorStop();
void motorSetLeft(float pwm);
void motorSetRight(float pwm);
// Gọi từ subscription /motor/*_target_rpm
void onLeftTarget(float rpm);
void onRightTarget(float rpm);
// Gọi mỗi vòng loop() — tự canh PID_PERIOD_MS
void motorUpdate();
// Đọc RPM đã lọc (debug)
float motorRpmLeft();
float motorRpmRight();

// motor_control.cpp — ruột PID inline của bạn, bọc lại cho gọn.
// Không đổi thuật toán: chỉ đổi cách tổ chức (kiểu Leonardo).
#include "motor_control.h"
#include "config.h"
#include <math.h>

namespace {
// PID state (mỗi bên riêng, như code gốc)
volatile float targetL = 0.0f, targetR = 0.0f;
float filtL = 0.0f, filtR = 0.0f;
float intL = 0.0f, intR = 0.0f;
float prevErrL = 0.0f, prevErrR = 0.0f;
unsigned long lastPidMs = 0;
unsigned long lastCmdMs = 0;
long prevCntL = 0, prevCntR = 0;
extern volatile long g_leftCount, g_rightCount;  // định nghĩa ở .ino

float pidStep(float target, float meas, float dt,
              float Kp, float Ki, float Kd, float &integ, float &prevErr) {
  float err = target - meas;
  integ += err * dt;
  if (integ > INTEGRAL_LIMIT) integ = INTEGRAL_LIMIT;
  if (integ < -INTEGRAL_LIMIT) integ = -INTEGRAL_LIMIT;
  float deriv = (err - prevErr) / dt;
  prevErr = err;
  float out = Kp * err + Ki * integ + Kd * deriv;
  if (out > MAX_PWM) out = MAX_PWM;
  if (out < -MAX_PWM) out = -MAX_PWM;
  return out;
}
}  // namespace

void motorInit() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CH_LEFT);
  ledcAttachPin(ENB, PWM_CH_RIGHT);
  motorStop();
  lastPidMs = millis();
  lastCmdMs = millis();
}

void motorSetLeft(float pwm) {
  if (pwm > MAX_PWM) pwm = MAX_PWM;
  if (pwm < -MAX_PWM) pwm = -MAX_PWM;
  if (pwm > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    ledcWrite(PWM_CH_LEFT, (int)pwm);
  } else if (pwm < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    ledcWrite(PWM_CH_LEFT, (int)(-pwm));
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    ledcWrite(PWM_CH_LEFT, 0);
  }
}

void motorSetRight(float pwm) {
  if (pwm > MAX_PWM) pwm = MAX_PWM;
  if (pwm < -MAX_PWM) pwm = -MAX_PWM;
  if (pwm > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    ledcWrite(PWM_CH_RIGHT, (int)pwm);
  } else if (pwm < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    ledcWrite(PWM_CH_RIGHT, (int)(-pwm));
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    ledcWrite(PWM_CH_RIGHT, 0);
  }
}

void motorStop() {
  motorSetLeft(0);
  motorSetRight(0);
  intL = 0; intR = 0;
  prevErrL = 0; prevErrR = 0;
}

void onLeftTarget(float rpm) { targetL = rpm; lastCmdMs = millis(); }
void onRightTarget(float rpm) { targetR = rpm; lastCmdMs = millis(); }
float motorRpmLeft() { return filtL; }
float motorRpmRight() { return filtR; }

void motorUpdate() {
  unsigned long now = millis();
  if (now - lastPidMs < PID_PERIOD_MS) return;
  float dt = (now - lastPidMs) / 1000.0f;
  lastPidMs = now;
  if (now - lastCmdMs > CMD_TIMEOUT_MS) { targetL = 0; targetR = 0; }

  long cl, cr;
  noInterrupts();
  cl = g_leftCount; cr = g_rightCount;
  interrupts();
  long dL = cl - prevCntL, dR = cr - prevCntR;
  prevCntL = cl; prevCntR = cr;
  if (dt > 0.001f) {
    float rawL = (dL / ENCODER_TPR) * 60.0f / dt;
    float rawR = (dR / ENCODER_TPR) * 60.0f / dt;
    // KHÔNG đảo dấu ở tầng RPM (bài học 29 RPM vọt 176 RPM của bạn).
    filtL = RPM_FILTER_ALPHA * rawL + (1.0f - RPM_FILTER_ALPHA) * filtL;
    filtR = RPM_FILTER_ALPHA * rawR + (1.0f - RPM_FILTER_ALPHA) * filtR;
  }
  if (fabs(targetL) < 0.5f && fabs(targetR) < 0.5f) { motorStop(); return; }

  motorSetLeft(pidStep(targetL, filtL, 0.1f, 1.8f, 2.5f, 0.15f, intL, prevErrL));
  motorSetRight(pidStep(targetR, filtR, 0.1f, 1.8f, 2.5f, 0.15f, intR, prevErrR));
}

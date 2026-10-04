// encoder.h — class Encoder kiểu Leonardo, ruột là luật dấu của bạn.
// Bạn đếm x1 (RISING chân A), 1 vòng = 234 xung.
#pragma once
#include <Arduino.h>
#include "config.h"

class KaiaEncoder {
public:
  KaiaEncoder(int pinA, int pinB, bool invert)
    : _pinA(pinA), _pinB(pinB), _invert(invert), _ticks(0) {}
  void begin(void (*isr)()) {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pinA), isr, RISING);
  }
  void IRAM_ATTR handleInterrupt() {
    bool A = digitalRead(_pinA);
    bool B = digitalRead(_pinB);
    // Luật của bạn: trái A==B ++ else -- ; phải ngược lại (A==B -- else ++).
    bool fwd = (A == B);
    if (_invert) fwd = !fwd;
    if (fwd) _ticks++; else _ticks--;
  }
  // Đọc + reset an toàn, trả RPM theo đúng dt đo thật (ms).
  float getRPMandReset(unsigned long dt_ms) {
    noInterrupts();
    long t = _ticks;
    _ticks = 0;
    interrupts();
    if (dt_ms == 0) return 0.0f;
    return (t * 60000.0f) / (ENCODER_TPR * dt_ms);
  }
  long rawCount() const { return _ticks; }
private:
  int _pinA, _pinB;
  bool _invert;
  volatile long _ticks;
};

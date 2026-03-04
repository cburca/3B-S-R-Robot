#include "pid.h"

void PID::setGains(float kp, float ki, float kd) {
  _kp = kp; _ki = ki; _kd = kd;
}

void PID::setLimits(float outMin, float outMax) {
  _outMin = outMin; _outMax = outMax;
}

void PID::reset() {
  _i = 0; _prevErr = 0;
}

float PID::update(float setpoint, float measurement, float dt) {
  float err = setpoint - measurement;
  _i += err * dt;
  float d = (err - _prevErr) / dt;
  _prevErr = err;

  float out = _kp * err + _ki * _i + _kd * d;
  if (out > _outMax) out = _outMax;
  if (out < _outMin) out = _outMin;
  return out;
}

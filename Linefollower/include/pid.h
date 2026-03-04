#pragma once

class PID {
public:
  void setGains(float kp, float ki, float kd);
  void setLimits(float outMin, float outMax);
  void reset();
  float update(float setpoint, float measurement, float dt);
private:
  float _kp, _ki, _kd;
  float _i, _prevErr;
  float _outMin, _outMax;
};
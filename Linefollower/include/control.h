#pragma once
#include "pid.h"
#include "encoders.h"
#include "motors.h"

class MotorController {
public:
  void begin(Encoders* enc, Motors* mot, float hz);
  void setTargets(float left, float right); // ticks/s
  void update(); // fixed-rate
  void stop();
private:
  PID _pidL, _pidR;
  Encoders* _enc;
  Motors* _mot;

  float _spL, _spR;
  int32_t _prevCountL, _prevCountR;
  uint32_t _lastMs;
  float _dt;
};
#include "control.h"
#include "config.h"
#include "pid.h"

void MotorController::begin(Encoders* enc, Motors* mot, float hz) {
  _enc = enc;
  _mot = mot;

  _pidL.setGains(KP, KI, KD);
  _pidR.setGains(KP, KI, KD);
  _pidL.setLimits(-255, 255);
  _pidR.setLimits(-255, 255);

  _spL = _spR = 0;
  _prevCountL = _prevCountR = 0;
  _lastMs = millis();
  _dt = 1.0 / hz; // to be tuned 
}

void MotorController::setTargets(float left, float right) {
  _spL = left;
  _spR = right;
}

void MotorController::update() {
  uint32_t now = millis();
  if (now - _lastMs < _dt * 1000) return; // not time yet
  _lastMs = now;

  int32_t countL, countR;
  _enc->readCounts(countL, countR);

  float measL = (countL - _prevCountL) / _dt;
  float measR = (countR - _prevCountR) / _dt;
  _prevCountL = countL;
  _prevCountR = countR;

  int16_t cmdL = (int16_t)_pidL.update(_spL, measL, _dt);
  int16_t cmdR = (int16_t)_pidR.update(_spR, measR, _dt);
  _mot->set(cmdL, cmdR);
}

void MotorController::stop() {
  _pidL.reset();
  _pidR.reset();
  setTargets(0, 0);
}


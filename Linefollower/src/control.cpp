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
  float dt = (now - _lastMs) * 0.001f;
  if (dt < _dt) return;
  _lastMs = now;

  int32_t countL, countR;
  _enc->readCounts(countL, countR);

  float measL = (countL - _prevCountL) / dt;
  float measR = (countR - _prevCountR) / dt;
  _prevCountL = countL;
  _prevCountR = countR;

  int16_t cmdL = (int16_t)_pidL.update(_spL, measL, dt);
  int16_t cmdR = (int16_t)_pidR.update(_spR, measR, dt);
  _mot->set(cmdL, cmdR);
}

void MotorController::stop() {
  _pidL.reset();
  _pidR.reset();
  setTargets(0, 0);
  _mot->set(0, 0);
  _enc->readCounts(_prevCountL, _prevCountR);
  _lastMs = millis();
}


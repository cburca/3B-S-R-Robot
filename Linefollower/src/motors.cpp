#include "motors.h"

Motors::Motors() {}

void Motors::begin(uint8_t lFwdPwm, uint8_t lRevPwm,
                   uint8_t rFwdPwm, uint8_t rRevPwm,
                   uint8_t enablePinL,
                   uint8_t enablePinR,
                   bool enableActiveHigh) {
  _L.fwd = lFwdPwm; _L.rev = lRevPwm;
  _R.fwd = rFwdPwm; _R.rev = rRevPwm;

  pinMode(_L.fwd, OUTPUT);
  pinMode(_L.rev, OUTPUT);
  pinMode(_R.fwd, OUTPUT);
  pinMode(_R.rev, OUTPUT);

  analogWrite(_L.fwd, 0);
  analogWrite(_L.rev, 0);
  analogWrite(_R.fwd, 0);
  analogWrite(_R.rev, 0);

  _enPinL = enablePinL;
  _enPinR = enablePinR;
  _enActiveHigh = enableActiveHigh;

  if (_enPinL != 255) pinMode(_enPinL, OUTPUT);
  if (_enPinR != 255) pinMode(_enPinR, OUTPUT);

  enable();
}

void Motors::setMaxAbsCmd(uint8_t maxAbs) {
  if (maxAbs == 0) maxAbs = 1;
  _maxAbs = maxAbs;
}

int16_t Motors::clampCmd(int16_t cmd, uint8_t maxAbs) {
  if (cmd > (int16_t)maxAbs) return (int16_t)maxAbs;
  if (cmd < -(int16_t)maxAbs) return -(int16_t)maxAbs;
  return cmd;
}

uint8_t Motors::toPwm(int16_t mag) {
  if (mag < 0) mag = -mag;
  if (mag > 255) mag = 255;
  return (uint8_t)mag;
}

void Motors::writeStop(const Chan& ch) {
  analogWrite(ch.fwd, 0);
  analogWrite(ch.rev, 0);
}

void Motors::writeChan(const Chan& ch, int16_t cmd) {
  cmd = clampCmd(cmd, _maxAbs);

  if (cmd == 0) {
    writeStop(ch);
    return;
  }

  const uint8_t pwm = toPwm(cmd);

  if (cmd > 0) {
    analogWrite(ch.fwd, pwm);
    analogWrite(ch.rev, 0);
  } else {
    analogWrite(ch.fwd, 0);
    analogWrite(ch.rev, pwm);
  }
}

void Motors::setLeft(int16_t cmd) {
  _lastLeft = clampCmd(cmd, _maxAbs);
  writeChan(_L, _lastLeft);
}

void Motors::setRight(int16_t cmd) {
  _lastRight = clampCmd(cmd, _maxAbs);
  writeChan(_R, _lastRight);
}

void Motors::set(int16_t leftCmd, int16_t rightCmd) {
  setLeft(leftCmd);
  setRight(rightCmd);
}

void Motors::setLeftNorm(float u) {
  if (u > 1.0f) u = 1.0f;
  if (u < -1.0f) u = -1.0f;
  setLeft((int16_t)(u * 255.0f));
}

void Motors::setRightNorm(float u) {
  if (u > 1.0f) u = 1.0f;
  if (u < -1.0f) u = -1.0f;
  setRight((int16_t)(u * 255.0f));
}

void Motors::setNorm(float leftU, float rightU) {
  setLeftNorm(leftU);
  setRightNorm(rightU);
}

void Motors::writeEnable(bool on) {
  const uint8_t val = (on ? (_enActiveHigh ? HIGH : LOW)
                          : (_enActiveHigh ? LOW : HIGH));
  if (_enPinL != 255) digitalWrite(_enPinL, val);
  if (_enPinR != 255) digitalWrite(_enPinR, val);
}

void Motors::enable() {
  writeEnable(true);
}

void Motors::disable() {
  stop();
  writeEnable(false);
}

void Motors::stop() {
  _lastLeft = 0;
  _lastRight = 0;
  writeStop(_L);
  writeStop(_R);
}
#pragma once
#include <Arduino.h>

class Motors {
public:
  Motors();

  void begin(uint8_t lFwdPwm, uint8_t lRevPwm,
             uint8_t rFwdPwm, uint8_t rRevPwm,
             uint8_t enablePinL = 255,
             uint8_t enablePinR = 255,
             bool enableActiveHigh = true);

  void setMaxAbsCmd(uint8_t maxAbs);

  void setLeft(int16_t cmd);
  void setRight(int16_t cmd);
  void set(int16_t leftCmd, int16_t rightCmd);

  void setLeftNorm(float u);
  void setRightNorm(float u);
  void setNorm(float leftU, float rightU);

  void enable();
  void disable();
  void stop();

private:
  struct Chan { uint8_t fwd, rev; };

  Chan _L{0, 0};
  Chan _R{0, 0};

  uint8_t _enPinL = 255;
  uint8_t _enPinR = 255;
  bool _enActiveHigh = true;

  uint8_t _maxAbs = 250;
  int16_t _lastLeft = 0;
  int16_t _lastRight = 0;

  static int16_t clampCmd(int16_t cmd, uint8_t maxAbs);
  static uint8_t toPwm(int16_t mag);

  void writeStop(const Chan& ch);
  void writeChan(const Chan& ch, int16_t cmd);
  void writeEnable(bool on);
};
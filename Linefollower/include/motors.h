#pragma once
#include <Arduino.h>

class Motors {
public:
  enum class StopMode : uint8_t {
    Coast = 0,   // both PWM = 0
  };

  Motors();

  void begin(uint8_t lFwdPwm, uint8_t lRevPwm,
             uint8_t rFwdPwm, uint8_t rRevPwm,
             uint8_t enablePin = 255, // add enablepin numbers
             bool enableActiveHigh = true);

  // + => forward (drives FWD PWM), - => reverse (drives REV PWM)
  void setLeft(int16_t cmd);
  void setRight(int16_t cmd);
  void set(int16_t leftCmd, int16_t rightCmd);

  void setLeftNorm(float u);
  void setRightNorm(float u);
  void setNorm(float leftU, float rightU);

  // Optional: set stop behavior used when cmd=0 or when calling stop()
  void setStopMode(StopMode mode);

  // Enable/disable driver (if enable pin provided)
  void enable();
  void disable();

  // Immediately stop both motors
  void stop();

  void setMaxAbsCmd(uint8_t maxAbs); // 1..255 / maybe we will hard code this to 255

  // For debugging / telemetry
  int16_t lastLeftCmd() const { return _lastLeft; }
  int16_t lastRightCmd() const { return _lastRight; }

private:
  struct Chan {
    uint8_t fwd = 255;
    uint8_t rev = 255;
  };

  Chan _L, _R;

  uint8_t _enPin = 255;
  bool _enActiveHigh = true;
  bool _hasEnable = false;

  StopMode _stopMode = StopMode::Coast;
  uint8_t _maxAbs = 255;

  int16_t _lastLeft = 0;
  int16_t _lastRight = 0;

  static int16_t clampCmd(int16_t cmd, uint8_t maxAbs);
  static uint8_t toPwm(int16_t mag);
  void writeChan(const Chan& ch, int16_t cmd);
  void writeStop(const Chan& ch);
};
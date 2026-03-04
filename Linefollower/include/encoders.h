#pragma once
#include <Arduino.h>

class Encoders {
public:
  void begin(uint8_t lA, uint8_t lB, int8_t lSign,
             uint8_t rA, uint8_t rB, int8_t rSign);

  void readCounts(int32_t &left, int32_t &right) const;
  void reset();

private:
  static volatile int32_t _leftCount;
  static volatile int32_t _rightCount;

  static uint8_t _lA, _lB, _rA, _rB;
  static int8_t  _lSign, _rSign;

  static int8_t readQuadDir(uint8_t aPin, uint8_t bPin);
  static void isrLeftA();
  static void isrRightA();
};
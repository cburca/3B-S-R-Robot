#include "encoders.h"

volatile int32_t Encoders::_leftCount = 0;
volatile int32_t Encoders::_rightCount = 0;

uint8_t Encoders::_lA = 0;
uint8_t Encoders::_lB = 0;
uint8_t Encoders::_rA = 0;
uint8_t Encoders::_rB = 0;

int8_t Encoders::_lSign = +1;
int8_t Encoders::_rSign = +1;

int8_t Encoders::readQuadDir(uint8_t aPin, uint8_t bPin) {
  bool a = digitalRead(aPin);
  bool b = digitalRead(bPin);
  return (a ^ b) ? +1 : -1;
}

// append encouder delta to count, direction determined by current state of A and B pins
void Encoders::isrLeftA() {
  _leftCount += (int32_t)(_lSign * readQuadDir(_lA, _lB));
}

void Encoders::isrLeftB() {
  _leftCount += (int32_t)(_lSign * readQuadDir(_lB, _lA));
}

void Encoders::isrRightA() {
  _rightCount += (int32_t)(_rSign * readQuadDir(_rA, _rB));
}

void Encoders::isrRightB() {
  _rightCount += (int32_t)(_rSign * readQuadDir(_rB, _rA));
}

void Encoders::begin(uint8_t lA, uint8_t lB, int8_t lSign,
                     uint8_t rA, uint8_t rB, int8_t rSign) {
  _lA = lA; _lB = lB; _lSign = lSign;
  _rA = rA; _rB = rB; _rSign = rSign;

  pinMode(_lA, INPUT_PULLUP);
  pinMode(_lB, INPUT_PULLUP);
  pinMode(_rA, INPUT_PULLUP);
  pinMode(_rB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(_lA), Encoders::isrLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_lB), Encoders::isrLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_rA), Encoders::isrRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_rB), Encoders::isrRightB, CHANGE);
}

void Encoders::readCounts(int32_t &left, int32_t &right) const {
  noInterrupts();
  left = _leftCount;
  right = _rightCount;
  interrupts();
}

void Encoders::reset() {
  noInterrupts();
  _leftCount = 0;
  _rightCount = 0;
  interrupts();
}
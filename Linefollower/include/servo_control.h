#pragma once
#include <Arduino.h>
#include <Servo.h>

class ServoControl {
public:
    void begin();
    bool pickSequence();
    bool placeSequence();

private:
    Servo SyringeServo;
    Servo LiftServo;
};

#include <Servo.h>
#include "servo_control.h"
#include "config.h"

void ServoControl::begin() {
    SyringeServo.attach(12);
    LiftServo.attach(13);

    // set to neutral position
}

bool ServoControl::pickSequence() {
    // Pick sequence implementation
    LiftServo.write(GRIP_POS);     //Lower
    delay(LOWER_DELAY);
    SyringeServo.write(GRIP_POS);   //Grip
    delay(GRIP_DELAY);
    LiftServo.write(LIFT_POS);     //Raise
    delay(RAISE_DELAY);

    return true; // return true once sequence is complete
}

bool ServoControl::placeSequence() {
    // Place sequence implementation
    LiftServo.write(LOWER_POS);     //Lower
    delay(LOWER_DELAY);
    SyringeServo.write(UNGRIP_POS);   //Release
    delay(GRIP_DELAY);
    LiftServo.write(LIFT_POS);     //Raise
    delay(RAISE_DELAY);

    return true;
}

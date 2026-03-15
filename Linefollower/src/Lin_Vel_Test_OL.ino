///////////////////////////////////////////////////////////////////////////
/////                                                                 /////
/////   Milestone 6 - Linear Velocity Specification Test, OPEN Loop   /////
/////                                                                 /////
///////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "config.h"
#include "motors.h"

// Objects
Motors motors;

// State
enum class DriveMode : uint8_t { CLOSED_LOOP_VEL, OPEN_LOOP_PWM };
static DriveMode driveMode = DriveMode::CLOSED_LOOP_VEL;

// For OPEN_LOOP_PWM mode
static int16_t openL = 0;
static int16_t openR = 0;


static void hardStop() {
  // Stop motors and reset controller
  motors.stop();
  openL = openR = 0;
}


// Arduino setup and loop
void setup() {
  Serial.begin(BAUD);

  // Motors
  motors.begin(L_FWD_PWM, L_REV_PWM,
               R_FWD_PWM, R_REV_PWM,
               MOTOR_EN_PIN,
               MOTOR_EN_ACTIVE_HIGH);

  motors.setStopMode(Motors::StopMode::Coast);
  motors.setMaxAbsCmd(250);

  hardStop();
  lastCmdMs = millis();

  Serial.println("READY");

  motors.set(250,250);
}

void loop() {
  delay(10000); // Max 10 second drive duration

  while(1); // Infinite loop
}
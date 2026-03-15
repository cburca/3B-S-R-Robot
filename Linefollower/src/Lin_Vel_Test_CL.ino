/////////////////////////////////////////////////////////////////////////////
/////                                                                   /////
/////   Milestone 6 - Linear Velocity Specification Test, CLOSED Loop   /////
/////                                                                   /////
/////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "control.h"

// Objects
Motors motors;
Encoders encoders;
MotorController ctrl;

// State
enum class DriveMode : uint8_t { CLOSED_LOOP_VEL, OPEN_LOOP_PWM };
static DriveMode driveMode = DriveMode::CLOSED_LOOP_VEL;

// For OPEN_LOOP_PWM mode
static int16_t openL = 0;
static int16_t openR = 0;

// Watchdog timestamp
static uint32_t lastCmdMs = 0;

static void hardStop() {
  // Stop motors and reset controller
  ctrl.stop();
  motors.stop();
  openL = openR = 0;
}

float velAt90PWM = 4.6; // <-- Get via experimentation; ** CONFIRM BEFORE USING **

// Arduino setup and loop
void setup() {
  Serial.begin(BAUD);

  // Bring up encoders (must set up ISRs inside Encoders::begin)
  encoders.begin(ENC_L_A, ENC_L_B, ENC_L_SIGN,
                 ENC_R_A, ENC_R_B, ENC_R_SIGN);

  // Motors
  motors.begin(L_FWD_PWM, L_REV_PWM,
               R_FWD_PWM, R_REV_PWM,
               MOTOR_EN_PIN,
               MOTOR_EN_ACTIVE_HIGH);

  motors.setStopMode(Motors::StopMode::Coast);
  motors.setMaxAbsCmd(250);

  // Controller
  ctrl.begin(&encoders, &motors, CTRL_HZ);
  ctrl.setTargets(0, 0);

  hardStop();
  lastCmdMs = millis();

  Serial.println("READY");

  ctrl.setTargets(velAt90PWM, velAt90PWM);
}

void loop() {
  // Max 10 second drive duration
  const uint32_t now = millis();
  if (now - lastCmdMs > 10000) {
    hardStop();
    lastCmdMs = now;
  }

  ctrl.update();
  }
#pragma once
#include <Arduino.h>
#include "motors.h"

// ++ SERIAL ++
static const uint32_t BAUD = 115200;

// telemetry period (ms)
static const uint16_t TELEMETRY_PERIOD_MS = 20;   // 50 Hz

// Command watchdog timeout (ms)
static const uint32_t CMD_TIMEOUT_MS = 250;

// ++ MOTORS (dual-PWM) ++
// Each side has TWO PWM pins: forward and reverse.
// ex) Motors::begin(lFwd, lRev, rFwd, rRev, enablePin, enableActiveHigh)

static const uint8_t L_FWD_PWM = 5;
static const uint8_t L_REV_PWM = 6;

static const uint8_t R_FWD_PWM = 9;
static const uint8_t R_REV_PWM = 10;

// motor dirver enable pins
static const uint8_t MOTOR_EN_PIN_L = 22;   // left channel enable
static const uint8_t MOTOR_EN_PIN_R = 24;  // right channel enable
static const bool    MOTOR_EN_ACTIVE_HIGH = true;

// limit max PWM magnitude (safety / tuning)
static const uint8_t MOTOR_MAX_ABS_CMD = 250;

// ++ ENCODERS ++
static const uint8_t ENC_L_A = 3;   // interrupt-capable
static const uint8_t ENC_L_B = 2;

static const uint8_t ENC_R_A = 19;   // interrupt-capable
static const uint8_t ENC_R_B = 18;

// flip if direction is inverted
static const int8_t ENC_L_SIGN = +1;
static const int8_t ENC_R_SIGN = +1;

static const int16_t ENCODER_CPR = 2797; // counts per revolution (CPR) of the encoders

// ++ CONTROL ++
static const float CTRL_HZ = 100.0f;

// PID Gains
static const float KP = 0.00194f;
static const float KI = 0.541f;
static const float KD = 0.0f;

// global variables for maneuvers
static const float KP_POSITION = 0.1f;   // Proportional gain
static const int16_t MIN_PWM = 50;    // Minimum PWM required to overcome motor friction
static const int16_t MAX_PWM = 100;   // Maximum turning speed
static const int32_t TOLERANCE = 40;  // Acceptable error in encoder counts
static const float WHEEL_RADIUS_M = 0.04f;   // meters
static const float TRACK_WIDTH_M  = 0.179f;   // meters, wheel-center to wheel-center
static const uint32_t SAMPLE_PERIOD_MS = 10;

static const float DROPOFF_DELTA = 0.250f;      // Dist from camera lense to center of balloon, in meters - JUST AN ESTIMATE, get from CAD + physical measurements
static const float TUNABLE_LIN_OFFSET = 0.0f;     // TUNE THIS VALUE
static const float TUNABLE_ANG_OFFSET = 0.0f;     // TUNE THIS VALUE


// Servos
static const uint8_t SYRINGE_SERVO_PIN = 12;
static const uint8_t LIFT_SERVO_PIN = 13;

static const uint8_t GRIP_POS = 15; // confirm with ryan
static const uint8_t UNGRIP_POS = 180;

static const uint8_t LIFT_POS = 0;
static const uint8_t LOWER_POS = 180;

static const uint16_t GRIP_DELAY = 500; // ms to wait after gripping/ungripping
static const uint16_t LOWER_DELAY = 500; // ms to wait after lowering
static const uint16_t RAISE_DELAY = 500; // ms to wait after raising
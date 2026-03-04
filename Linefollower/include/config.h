#pragma once
#include <Arduino.h>
#include "motors.h"

// -------------------- Serial --------------------
static const uint32_t BAUD = 115200;

// telemetry period (ms)
static const uint16_t TELEMETRY_PERIOD_MS = 20;   // 50 Hz

// Command watchdog timeout (ms)
static const uint32_t CMD_TIMEOUT_MS = 250;

// -------------------- Motors (dual-PWM H-bridge) --------------------
// Each side has TWO PWM pins: forward and reverse.
// ex) Motors::begin(lFwd, lRev, rFwd, rRev, enablePin, enableActiveHigh)

static const uint8_t L_FWD_PWM = 5;
static const uint8_t L_REV_PWM = 6;

static const uint8_t R_FWD_PWM = 9;
static const uint8_t R_REV_PWM = 10;

// motor driver enable pin.
static const uint8_t MOTOR_EN_PIN = 22;
static const bool    MOTOR_EN_ACTIVE_HIGH = true;

// Limit max PWM magnitude (safety / tuning)
static const uint8_t MOTOR_MAX_ABS_CMD = 255;

// -------------------- Encoders (quadrature) --------------------
// Encoders::begin(lA, lB, lSign, rA, rB, rSign)
static const uint8_t ENC_L_A = 2;   // interrupt-capable
static const uint8_t ENC_L_B = 4;

static const uint8_t ENC_R_A = 3;   // interrupt-capable
static const uint8_t ENC_R_B = 7;

// Flip if direction is inverted
static const int8_t ENC_L_SIGN = +1;
static const int8_t ENC_R_SIGN = +1;

// -------------------- Control --------------------
// Controller update frequency (Hz)
static const float CTRL_HZ = 100.0f;

// PID Gains (ticks/sec control)
static const float KP = 1.0f;
static const float KI = 0.1f;
static const float KD = 0.01f;
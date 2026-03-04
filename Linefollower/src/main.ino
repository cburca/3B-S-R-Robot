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

// -------------------- Serial line buffer --------------------
static char lineBuf[80];
static uint8_t lineLen = 0;

// -------------------- Helpers --------------------
static inline int16_t clampI16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}

static void hardStop() {
  // Stop motors and reset controller
  ctrl.stop();
  motors.stop();
  openL = openR = 0;
}

// Parse one complete line command
static void handleLine(char* s) {
  // Skip leading spaces
  while (*s == ' ' || *s == '\t') s++;
  if (*s == '\0') return;

  // Command letter
  char c = *s++;

  // Skip spaces after command
  while (*s == ' ' || *s == '\t') s++;

  if (c == 'V' || c == 'v') {
    // Velocity targets: ticks/sec
    float l = 0, r = 0;
    if (sscanf(s, "%f %f", &l, &r) == 2) {
      driveMode = DriveMode::CLOSED_LOOP_VEL;
      ctrl.setTargets(l, r);
      lastCmdMs = millis();
    }
    return;
  }

  if (c == 'P' || c == 'p') {
    // Raw PWM: -255..255
    int32_t l = 0, r = 0;
    if (sscanf(s, "%ld %ld", &l, &r) == 2) {
      driveMode = DriveMode::OPEN_LOOP_PWM;
      openL = clampI16(l, -255, 255);
      openR = clampI16(r, -255, 255);
      lastCmdMs = millis();
    }
    return;
  }

  if (c == 'S' || c == 's') {
    hardStop();
    lastCmdMs = millis();
    return;
  }

  if (c == 'E' || c == 'e') {
    int en = 1;
    if (sscanf(s, "%d", &en) >= 1) {
      if (en) motors.enable();
      else motors.disable();
      lastCmdMs = millis();
    }
    return;
  }
}

// Non-blocking serial line reader
static void pollSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();

    if (ch == '\r') continue; // ignore CR

    if (ch == '\n') {
      // Terminate and handle
      lineBuf[lineLen] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
    } else {
      // Append if space
      if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = ch;
      } else {
        // Overflow -> reset buffer
        lineLen = 0;
      }
    }
  }
}

// Arduino setup and loop
void setup() {
  Serial.begin(115200);

  // Bring up encoders (must set up ISRs inside Encoders::begin)
  encoders.begin(ENC_L_A, ENC_L_B, ENC_L_SIGN,
                 ENC_R_A, ENC_R_B, ENC_R_SIGN);

  // Motors
  motors.begin(L_FWD_PWM, L_REV_PWM,
               R_FWD_PWM, R_REV_PWM,
               MOTOR_EN_PIN,
               MOTOR_EN_ACTIVE_HIGH);

  motors.setStopMode(Motors::StopMode::Coast);
  motors.setMaxAbsCmd(255);

  // Controller
  ctrl.begin(&encoders, &motors, CTRL_HZ);
  ctrl.setTargets(0, 0);

  hardStop();
  lastCmdMs = millis();

  Serial.println("READY");
}

void loop() {
  pollSerial();

  // Watchdog safety
  const uint32_t now = millis();
  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    hardStop();
    // keep lastCmdMs from wrapping the stop repeatedly
    lastCmdMs = now;
  }

  // Drive update
  if (driveMode == DriveMode::CLOSED_LOOP_VEL) {
    ctrl.update();   // internally rate-limited by _dt
  } else {
    motors.set(openL, openR);
  }

  // Optional: tiny yield
  // delay(0);
}
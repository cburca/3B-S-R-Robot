#include <Arduino.h>
#include <stdlib.h>

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

// Serial line buffer
static char lineBuf[80];
static uint8_t lineLen = 0;

static constexpr float TWO_PI_F = 6.28318530718f;

static inline int16_t clampI16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}

static inline bool isSIX(const char* s) {
  return (s[0] == 'S' || s[0] == 's') &&
         (s[1] == 'I' || s[1] == 'i') &&
         (s[2] == 'X' || s[2] == 'x') &&
         (s[3] == '\0');
}

static bool parse2f(const char* s, float& a, float& b) {
  while (*s == ' ' || *s == '\t') s++;

  char* end1;
  double da = strtod(s, &end1);
  if (end1 == s) return false;

  while (*end1 == ' ' || *end1 == '\t') end1++;

  char* end2;
  double db = strtod(end1, &end2);
  if (end2 == end1) return false;

  a = (float)da;
  b = (float)db;
  return true;
}

static bool parse1i(const char* s, int& v) {
  while (*s == ' ' || *s == '\t') s++;
  char* end;
  long x = strtol(s, &end, 10);
  if (end == s) return false;
  v = (int)x;
  return true;
}

static bool parse2i(const char* s, int32_t& a, int32_t& b) {
  while (*s == ' ' || *s == '\t') s++;

  char* end1;
  long da = strtol(s, &end1, 10);
  if (end1 == s) return false;

  while (*end1 == ' ' || *end1 == '\t') end1++;

  char* end2;
  long db = strtol(end1, &end2, 10);
  if (end2 == end1) return false;

  a = (int32_t)da;
  b = (int32_t)db;
  return true;
}

static void hardStop() {
  ctrl.setTargets(0, 0);
  ctrl.stop();
  motors.stop();
  openL = openR = 0;
  driveMode = DriveMode::OPEN_LOOP_PWM;
}

// Parse one complete line command
static void handleLine(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  if (*s == '\0') return;

  if (isSIX(s)) {
    Serial.println("SEVEN");
    lastCmdMs = millis();
    return;
  }

  char c = *s++;
  while (*s == ' ' || *s == '\t') s++;

  if (c == 'V' || c == 'v') {
    float l = 0, r = 0;
    if (parse2f(s, l, r)) {
      driveMode = DriveMode::CLOSED_LOOP_VEL;
      ctrl.setTargets(l, r);
      lastCmdMs = millis();
      Serial.print("OK V ");
      Serial.print(l, 3);
      Serial.print(' ');
      Serial.println(r, 3);
    } else {
      Serial.print("ERR V s='");
      Serial.print(s);
      Serial.println("'");
    }
    return;
  }

  if (c == 'W' || c == 'w') {
    float wl = 0, wr = 0;
    if (parse2f(s, wl, wr)) {
      const float k = (float)ENCODER_CPR / TWO_PI_F;
      float l_tps = wl * k;
      float r_tps = wr * k;

      driveMode = DriveMode::CLOSED_LOOP_VEL;
      ctrl.setTargets(l_tps, r_tps);
      lastCmdMs = millis();

      Serial.print("OK W ");
      Serial.print(wl, 4);
      Serial.print(' ');
      Serial.println(wr, 4);
    } else {
      Serial.print("ERR W s='");
      Serial.print(s);
      Serial.println("'");
    }
    return;
  }

  if (c == 'P' || c == 'p') {
    int32_t l = 0, r = 0;
    if (parse2i(s, l, r)) {
      driveMode = DriveMode::OPEN_LOOP_PWM;
      openL = clampI16(l, -255, 255);
      openR = clampI16(r, -255, 255);
      lastCmdMs = millis();
      Serial.print("OK P ");
      Serial.print(openL);
      Serial.print(' ');
      Serial.println(openR);
    } else {
      Serial.print("ERR P s='");
      Serial.print(s);
      Serial.println("'");
    }
    return;
  }

  if (c == 'S' || c == 's') {
    hardStop();
    lastCmdMs = millis();
    Serial.println("OK S");
    return;
  }

  if (c == 'E' || c == 'e') {
    int en = 1;
    if (parse1i(s, en)) {
      if (en) motors.enable();
      else motors.disable();
      lastCmdMs = millis();
      Serial.print("OK E ");
      Serial.println(en ? 1 : 0);
    } else {
      Serial.print("ERR E s='");
      Serial.print(s);
      Serial.println("'");
    }
    return;
  }

  Serial.print("ERR ?");
  Serial.println(c);
}

// Non-blocking serial line reader
static void pollSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();

    if (ch == '\r') continue;

    if (ch == '\n') {
      lineBuf[lineLen] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
    } else {
      if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = ch;
      else lineLen = 0;
    }
  }
}

void setup() {
  Serial.begin(BAUD);

  encoders.begin(ENC_L_A, ENC_L_B, ENC_L_SIGN,
                 ENC_R_A, ENC_R_B, ENC_R_SIGN);

  motors.begin(L_FWD_PWM, L_REV_PWM,
               R_FWD_PWM, R_REV_PWM,
               MOTOR_EN_PIN_L, MOTOR_EN_PIN_R,
               MOTOR_EN_ACTIVE_HIGH);

  motors.setMaxAbsCmd(255);

  ctrl.begin(&encoders, &motors, CTRL_HZ);
  ctrl.setTargets(0, 0);

  hardStop();
  lastCmdMs = millis();

  Serial.println("READY");
}

void loop() {
  pollSerial();

  const uint32_t now = millis();
  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    hardStop();
    lastCmdMs = now;
  }

  if (driveMode == DriveMode::CLOSED_LOOP_VEL) {
    ctrl.update();
  } else {
    motors.set(openL, openR);
  }
}
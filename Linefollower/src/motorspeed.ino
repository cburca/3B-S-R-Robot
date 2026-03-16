#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"

Motors motors;
Encoders encoders;

static const float STEP_VOLTAGE = 10.8f;
static const uint32_t START_DELAY_MS = 2000;
static const uint32_t RUN_TIME_MS = 3000;
static const uint32_t SAMPLE_PERIOD_MS = 5;
static const float CPR = 2797.0f;

static int16_t voltageToCmd(float volts) {
  if (volts > 12.0f) volts = 12.0f;
  if (volts < -12.0f) volts = -12.0f;
  return (int16_t)(255.0f * volts / 12.0f);
}

static float cmdToVoltage(int16_t cmd) {
  return 12.0f * ((float)cmd / 255.0f);
}

void setup() {
  Serial.begin(BAUD);

  motors.begin(
    L_FWD_PWM,
    L_REV_PWM,
    R_FWD_PWM,
    R_REV_PWM,
    MOTOR_EN_PIN_L,
    MOTOR_EN_PIN_R,
    MOTOR_EN_ACTIVE_HIGH
  );

  motors.setMaxAbsCmd(MOTOR_MAX_ABS_CMD);
  motors.stop();

  encoders.begin(
    ENC_L_A, ENC_L_B, ENC_L_SIGN,
    ENC_R_A, ENC_R_B, ENC_R_SIGN
  );
  encoders.reset();

  delay(START_DELAY_MS);

  int16_t cmd = voltageToCmd(STEP_VOLTAGE);
  motors.set(cmd, 0);

  Serial.println("t_ms,cmd,v_eq,left_count,left_rad_s,left_rpm");
}

void loop() {
  static bool initialized = false;
  static bool running = true;
  static uint32_t startMs = 0;
  static uint32_t lastSampleMs = 0;
  static int32_t prevLeft = 0;

  if (!initialized) {
    int32_t dummyRight;
    startMs = millis();
    lastSampleMs = startMs;
    encoders.readCounts(prevLeft, dummyRight);
    initialized = true;
  }

  if (!running) return;

  uint32_t now = millis();

  if (now - startMs >= RUN_TIME_MS) {
    motors.stop();
    motors.disable();
    running = false;
    return;
  }

  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    int32_t leftCount, rightCount;
    encoders.readCounts(leftCount, rightCount);

    float dt = (now - lastSampleMs) * 0.001f;
    float leftCps = (leftCount - prevLeft) / dt;
    float leftRadS = leftCps * (2.0f * PI / CPR);
    float leftRpm = leftCps * (60.0f / CPR);

    int16_t cmd = voltageToCmd(STEP_VOLTAGE);
    float vEq = cmdToVoltage(cmd);

    Serial.print(now - startMs);
    Serial.print(',');
    Serial.print(cmd);
    Serial.print(',');
    Serial.print(vEq, 3);
    Serial.print(',');
    Serial.print(leftCount);
    Serial.print(',');
    Serial.print(leftRadS, 4);
    Serial.print(',');
    Serial.println(leftRpm, 3);

    prevLeft = leftCount;
    lastSampleMs = now;
  }
}
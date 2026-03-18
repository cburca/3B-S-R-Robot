// #include <Arduino.h>
// #include "config.h"
// #include "motors.h"
// #include "encoders.h"

// Motors motors;
// Encoders encoders;

// static const float TURN_DEG = 360.0f;
// static const float CPR = 2797.0f;

// // Replace these with your actual robot values
// static const float WHEEL_RADIUS_M = 0.04f;   // meters
// static const float TRACK_WIDTH_M  = 0.179f;   // meters, wheel-center to wheel-center

// static const int16_t TURN_CMD = 140;
// static const uint32_t START_DELAY_MS = 2000;
// static const uint32_t SAMPLE_PERIOD_MS = 10;

// static float degToRad(float deg) {
//   return deg * PI / 180.0f;
// }

// static int32_t computeTurnCounts(float turnDeg, float trackWidth, float wheelRadius, float cpr) {
//   float theta = degToRad(turnDeg);
//   float counts = theta * trackWidth * cpr / (4.0f * PI * wheelRadius);
//   return (int32_t)(counts + 0.5f);
// }

// void setup() {
//   Serial.begin(BAUD);

//   motors.begin(
//     L_FWD_PWM,
//     L_REV_PWM,
//     R_FWD_PWM,
//     R_REV_PWM,
//     MOTOR_EN_PIN_L,
//     MOTOR_EN_PIN_R,
//     MOTOR_EN_ACTIVE_HIGH
//   );

//   motors.setMaxAbsCmd(MOTOR_MAX_ABS_CMD);
//   motors.stop();

//   encoders.begin(
//     ENC_L_A, ENC_L_B, ENC_L_SIGN,
//     ENC_R_A, ENC_R_B, ENC_R_SIGN
//   );
//   encoders.reset();

//   delay(START_DELAY_MS);

//   Serial.println("t_ms,left_delta,right_delta,left_abs,right_abs,target_counts");
// }

// void loop() {
//   static bool initialized = false;
//   static bool running = true;
//   static uint32_t startMs = 0;
//   static uint32_t lastSampleMs = 0;
//   static int32_t startLeft = 0;
//   static int32_t startRight = 0;
//   static int32_t targetCounts = 0;

//   if (!initialized) {
//     encoders.readCounts(startLeft, startRight);
//     //targetCounts = computeTurnCounts(TURN_DEG, TRACK_WIDTH_M, WHEEL_RADIUS_M, CPR);

//     targetCounts = 2200;    // 1 Revolution = 2797

//     startMs = millis();
//     lastSampleMs = startMs;

//     // Spot turn: left forward, right backward
//     motors.set(TURN_CMD, -TURN_CMD);

//     Serial.print("Target counts per wheel: ");
//     Serial.println(targetCounts);

//     initialized = true;
//   }

//   if (!running) return;

//   uint32_t now = millis();

//   if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
//     int32_t leftCount, rightCount;
//     encoders.readCounts(leftCount, rightCount);

//     int32_t leftDelta = leftCount - startLeft;
//     int32_t rightDelta = rightCount - startRight;

//     int32_t leftAbs = abs(leftDelta);
//     int32_t rightAbs = abs(rightDelta);

//     Serial.print(now - startMs);
//     Serial.print(',');
//     Serial.print(leftDelta);
//     Serial.print(',');
//     Serial.print(rightDelta);
//     Serial.print(',');
//     Serial.print(leftAbs);
//     Serial.print(',');
//     Serial.print(rightAbs);
//     Serial.print(',');
//     Serial.println(targetCounts);

//     // Stop once both wheels have reached their theoretical count magnitude
//     if (leftAbs >= targetCounts || rightAbs >= targetCounts) {
//       motors.stop();
//       motors.disable();
//       running = false;

//       Serial.println("Turn complete");
//       Serial.print("Final left delta: ");
//       Serial.println(leftDelta);
//       Serial.print("Final right delta: ");
//       Serial.println(rightDelta);
//     }

//     lastSampleMs = now;
//   }
// }
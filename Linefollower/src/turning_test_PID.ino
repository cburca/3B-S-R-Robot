// #include <Arduino.h>
// #include "config.h"
// #include "motors.h"
// #include "encoders.h"

// Motors motors;
// Encoders encoders;

// /////////////////////////////////////////////////////

// static const float KP_turn = 0.1f;   // Proportional gain
// static const int16_t MIN_PWM = 50;    // Minimum PWM required to overcome motor friction
// static const int16_t MAX_PWM = 100;   // Maximum turning speed
// static const int32_t TOLERANCE = 40;  // Acceptable error in encoder counts

// /////////////////////////////////////////////////////

// static const float TURN_DEG = 360.0f;
// static const float CPR = 2797.0f;

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
    
//     targetCounts = computeTurnCounts(180.0f, TRACK_WIDTH_M, WHEEL_RADIUS_M, CPR);

//     startMs = millis();
//     lastSampleMs = startMs;

//     Serial.print("Target counts per wheel: ");
//     Serial.println(targetCounts);
//     initialized = true;
//   }

//   if (!running) return;

//   uint32_t now = millis();
//   if (now - lastSampleMs >= SAMPLE_PERIOD_MS) { 
//     int32_t leftCount, rightCount;
//     encoders.readCounts(leftCount, rightCount);

//     // Calculate absolute distance traveled
//     int32_t leftAbs = abs(leftCount - startLeft);
//     int32_t rightAbs = abs(rightCount - startRight);

//     // Calculate remaining error
//     int32_t errorL = targetCounts - leftAbs;
//     int32_t errorR = targetCounts - rightAbs;

//     // Calculate Proportional PWM
//     int16_t pwmL = errorL * KP_turn;
//     int16_t pwmR = errorR * KP_turn;

//     // Constrain PWM to safe maximums, but ensure it's high enough to move
//     if (pwmL > 0) pwmL = constrain(pwmL, MIN_PWM, MAX_PWM);
//     if (pwmR > 0) pwmR = constrain(pwmR, MIN_PWM, MAX_PWM);

//     // Stop wheels individually if they are within tolerance
//     if (errorL <= TOLERANCE) pwmL = 0;
//     if (errorR <= TOLERANCE) pwmR = 0;

//     // Apply motor speeds
//     motors.set(pwmL, -pwmR);

//     // Stop completely ONLY when BOTH wheels have reached the target
//     if (errorL <= TOLERANCE && errorR <= TOLERANCE) {
//       motors.stop(); // 
//       running = false; // [cite: 20]

//       Serial.println("Turn complete");
//       Serial.print("Final left absolute: ");
//       Serial.println(leftAbs);
//       Serial.print("Final right absolute: ");
//       Serial.println(rightAbs);
//     }

//     lastSampleMs = now;
//   }
// }
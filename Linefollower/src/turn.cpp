#include "turn.h"
#include "config.h"
#include "motors.h"
#include "encoders.h"

// global variables (from turning_test_PID.ino)
static const float KP_TURN = 0.1f;   // Proportional gain
static const int16_t MIN_PWM = 50;    // Minimum PWM required to overcome motor friction
static const int16_t MAX_PWM = 100;   // Maximum turning speed
static const int32_t TOLERANCE = 40;  // Acceptable error in encoder counts
static const float CPR = 2797.0f;
static const float WHEEL_RADIUS_M = 0.04f;   // meters
static const float TRACK_WIDTH_M  = 0.179f;   // meters, wheel-center to wheel-center
static const uint32_t SAMPLE_PERIOD_MS = 10;

static float degToRad(float deg) {
  return deg * PI / 180.0f;
}

static int32_t computeTurnCounts(float turn_deg) {
  float theta = degToRad(turn_deg);
  float counts = theta * TRACK_WIDTH_M * CPR / (4.0f * PI * WHEEL_RADIUS_M);
  return (int32_t)(counts + 0.5f);
}

void turnDegrees(float turn) {
    int32_t start_left, start_right;
    encoders.readCounts(start_left, start_right);

    int32_t target_counts = computeTurnCounts(turn_deg);

    bool running = true;
    uint32_t last_sample_ms = millis();

    while (running) {
        uint32_t now = millis();
        if (now - last_sample_ms < SAMPLE_PERIOD_MS) continue:
        last_sample_ms = now;

        int32_t left_count, right_count;
        encoders.readCounts(left_count, right_count);
        
        // calculate absolute distance travelled
        int32_t left_abs = abs(left_count - start_left);
        int32_t right_abs = abs(right_count - start_right);
        
        // calculate remaining error
        int32_t error_left = target_counts - left_abs;
        int32_t error_right = target_counts - right_abs;

        // calculate proportional PWM
        int16_t pwm_left = error_left * KP_TURN;
        int16_t pwm_right = error_right * KP_TURN;

        // limit PWM to safe max but still high enough to move
        if (pwm_left > 0) pwm_left = constrain(pwm_left, MIN_PWM, MAX_PWM);
        if (pwm_right > 0) pwm_right = constrain(pwm_right, MIN_PWM, MAX_PWM);

        // stop wheels individually if they are within tolerance
        if (error_left <= TOLERANCE) pwm_left = 0;
        if (error_right <= TOLERANCE) pwm_right = 0;

        // apply motor speed
        motors.set(pwm_left, -pwm_right);

        // stop fully only when both wheels reach target
        if (error_left <= TOLERANCE && error_right <= TOLERANCE) {
            motors.stop();
            running = false;
        }
    }
}
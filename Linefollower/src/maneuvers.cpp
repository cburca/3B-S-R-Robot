#include "maneuvers.h"
#include "config.h"
#include "motors.h"
#include "encoders.h"

#include <math.h>

static float degToRad(float deg) {
    return deg * PI / 180.0f;
}

static int32_t abs32(int32_t x) {
    return (x < 0) ? -x : x;
}

static int32_t computeTurnCounts(float turn_deg) {
    float theta = degToRad(fabsf(turn_deg));
    float counts = theta * TRACK_WIDTH_M * ENCODER_CPR / (4.0f * PI * WHEEL_RADIUS_M);
    return (int32_t)(counts + 0.5f);
}

static int32_t computeDropOffCounts(float offset_m) {
    float distance = offset_m + DROPOFF_DELTA;
    if (distance < 0.0f) {
        distance = 0.0f;
    }

    float counts = distance * ENCODER_CPR / (2.0f * PI * WHEEL_RADIUS_M);
    return (int32_t)(counts + 0.5f);
}

bool turnDegrees(float turn_deg, Motors& motors, Encoders& encoders) {
    if (fabsf(turn_deg) <= 1e-3f) {
        motors.stop();
        return true;
    }

    int32_t start_left, start_right;
    encoders.readCounts(start_left, start_right);

    const int8_t dir = (turn_deg >= 0.0f) ? 1 : -1;
    const int32_t target_counts = computeTurnCounts(turn_deg);

    const uint32_t start_ms = millis();
    uint32_t last_sample_ms = start_ms;

    while (true) {
        uint32_t now = millis();

        if (now - last_sample_ms < SAMPLE_PERIOD_MS) {
            continue;
        }
        last_sample_ms = now;

        if (now - start_ms > TURN_TIMEOUT_MS) {
            motors.stop();
            return false;
        }

        int32_t left_count, right_count;
        encoders.readCounts(left_count, right_count);

        int32_t left_travel = abs32(left_count - start_left);
        int32_t right_travel = abs32(right_count - start_right);

        int32_t error_left = target_counts - left_travel;
        int32_t error_right = target_counts - right_travel;

        int16_t pwm_left = 0;
        int16_t pwm_right = 0;

        if (error_left > TOLERANCE) {
            float cmd_left = error_left * KP_POSITION;
            if (cmd_left < MIN_PWM) cmd_left = MIN_PWM;
            if (cmd_left > MAX_PWM) cmd_left = MAX_PWM;
            pwm_left = (int16_t)cmd_left;
        }

        if (error_right > TOLERANCE) {
            float cmd_right = error_right * KP_POSITION;
            if (cmd_right < MIN_PWM) cmd_right = MIN_PWM;
            if (cmd_right > MAX_PWM) cmd_right = MAX_PWM;
            pwm_right = (int16_t)cmd_right;
        }

        motors.set(dir * pwm_left, -dir * pwm_right);

        if (error_left <= TOLERANCE && error_right <= TOLERANCE) {
            motors.stop();
            return true;
        }
    }
}
bool dropOff(Motors& motors, Encoders& encoders) {
    int32_t start_left, start_right;
    encoders.readCounts(start_left, start_right);

    float tunable_offset = TUNABLE_LIN_OFFSET; // Adjust for systematic errors

    const int32_t target_counts = computeDropOffCounts(tunable_offset);

    const uint32_t start_ms = millis();
    uint32_t last_sample_ms = start_ms;

    while (true) {
        uint32_t now = millis();
        if (now - last_sample_ms < SAMPLE_PERIOD_MS) {
            continue;
        }
        last_sample_ms = now;

        if (now - start_ms > SAMPLE_PERIOD_MS) {
            motors.stop();
            return false;
        }

        int32_t left_count, right_count;
        encoders.readCounts(left_count, right_count);

        int32_t left_travel = abs32(left_count - start_left);
        int32_t right_travel = abs32(right_count - start_right);

        int32_t error_left = target_counts - left_travel;
        int32_t error_right = target_counts - right_travel;

        int16_t pwm_left = 0;
        int16_t pwm_right = 0;

        if (error_left > TOLERANCE) {
            float cmd_left = error_left * KP_POSITION;
            if (cmd_left < MIN_PWM) cmd_left = MIN_PWM;
            if (cmd_left > MAX_PWM) cmd_left = MAX_PWM;
            pwm_left = (int16_t)cmd_left;
        }

        if (error_right > TOLERANCE) {
            float cmd_right = error_right * KP_POSITION;
            if (cmd_right < MIN_PWM) cmd_right = MIN_PWM;
            if (cmd_right > MAX_PWM) cmd_right = MAX_PWM;
            pwm_right = (int16_t)cmd_right;
        }

        motors.set(pwm_left, pwm_right);

        if (error_left <= TOLERANCE && error_right <= TOLERANCE) {
            motors.stop();
            return true;
        }
    }
}
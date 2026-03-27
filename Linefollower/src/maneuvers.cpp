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

static int32_t computeDropOffCounts() {
    float distance = DROPOFF_DELTA + TUNABLE_LIN_OFFSET;
    if (distance < 0.0f) {
        distance = 0.0f;
    }

    float counts = distance * ENCODER_CPR / (2.0f * PI * WHEEL_RADIUS_M);
    return (int32_t)(counts + 0.5f);
}

static int16_t positionToPwm(int32_t error_counts) {
    if (error_counts <= TOLERANCE) {
        return 0;
    }

    float cmd = error_counts * KP_POSITION;

    if (cmd < MIN_PWM) cmd = MIN_PWM;
    if (cmd > MAX_PWM) cmd = MAX_PWM;

    return (int16_t)cmd;
}

static bool runManeuver(
    int32_t target_left_counts,
    int32_t target_right_counts,
    int8_t left_dir,
    int8_t right_dir,
    uint32_t timeout_ms,
    Motors& motors,
    Encoders& encoders
) {
    int32_t start_left, start_right;
    encoders.readCounts(start_left, start_right);

    const uint32_t start_ms = millis();
    uint32_t last_sample_ms = start_ms;

    while (true) {
        uint32_t now = millis();

        if (now - start_ms > timeout_ms) {
            motors.stop();
            return false;
        }

        if (now - last_sample_ms < SAMPLE_PERIOD_MS) {
            continue;
        }
        last_sample_ms = now;

        int32_t left_count, right_count;
        encoders.readCounts(left_count, right_count);

        int32_t left_travel = abs32(left_count - start_left);
        int32_t right_travel = abs32(right_count - start_right);

        int32_t error_left = target_left_counts - left_travel;
        int32_t error_right = target_right_counts - right_travel;

        int16_t pwm_left = positionToPwm(error_left);
        int16_t pwm_right = positionToPwm(error_right);

        motors.set(left_dir * pwm_left, right_dir * pwm_right);

        if (error_left <= TOLERANCE && error_right <= TOLERANCE) {
            motors.stop();
            delay(75);
            return true;
        }
    }
}

bool turnDegrees(float turn_deg, Motors& motors, Encoders& encoders) {
    const float commanded_deg = turn_deg + copysignf(TUNABLE_ANG_OFFSET, turn_deg);

    if (fabsf(commanded_deg) <= 1e-3f) {
        motors.stop();
        return true;
    }

    const int8_t dir = (commanded_deg >= 0.0f) ? 1 : -1;
    const int32_t target_counts = computeTurnCounts(commanded_deg);

    return runManeuver(
        target_counts,
        target_counts,
        dir,
        -dir,
        TURN_TIMEOUT_MS,
        motors,
        encoders
    );
}

bool dropOff(Motors& motors, Encoders& encoders) {
    const int32_t target_counts = computeDropOffCounts();

    if (target_counts <= TOLERANCE) {
        motors.stop();
        return true;
    }

    return runManeuver(
        target_counts,
        target_counts,
        1,
        1,
        DROPOFF_TIMEOUT_MS,
        motors,
        encoders
    );
}
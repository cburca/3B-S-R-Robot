#pragma once

#include <Arduino.h>
#include "motors.h"
#include "encoders.h"

static const int SAMPLE_TIMEOUT_MS = 1000;

void turnDegrees(float turn_deg, Motors& motors, Encoders& encoders);
void dropOff(float tunable_offset, Motors& motors, Encoders& encoders);
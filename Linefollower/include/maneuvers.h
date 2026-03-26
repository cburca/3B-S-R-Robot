#pragma once

#include <Arduino.h>

static float degToRad(float deg);
static int32_t computeTurnCounts(float turn_deg);
void turnDegrees(float deg, Motors& motors, Encoders& encoders);

static int32_t computeDropOffCounts(float offset);
void dropOff(float extra_offset, Motors& motors, Encoders& encoders);
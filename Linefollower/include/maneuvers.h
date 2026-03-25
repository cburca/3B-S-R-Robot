#pragma once

#include <Arduino.h>

extern Motors motors;
extern Encoders encoders;

static float degToRad(float deg);
static int32_t computeTurnCounts(float turn_deg);
void turnDegrees(float deg);

static int32_t computeDropOffCounts(float offset);
void dropOff(float extra_offset);
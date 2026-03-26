#pragma once

#include <Arduino.h>
#include "motors.h"
#include "encoders.h"

float degToRad(float deg);
int32_t computeTurnCounts(float turn_deg);
void turnDegrees(float deg, Motors& motors, Encoders& encoders);

uint32_t computeDropOffCounts(float offset);
void dropOff(float extra_offset, Motors& motors, Encoders& encoders);

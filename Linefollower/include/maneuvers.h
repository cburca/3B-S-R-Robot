#pragma once

#include <Arduino.h>
#include "motors.h"
#include "encoders.h"
#include "config.h"

// static const int SAMPLE_TIMEOUT_MS = 1000; // DONT THINK WE NEED or USE 

void turnDegrees(float turn_deg, Motors& motors, Encoders& encoders);
void dropOff(Motors& motors, Encoders& encoders);
#pragma once

#include <Arduino.h>
#include "motors.h"
#include "encoders.h"

bool turnDegrees(float deg, Motors& motors, Encoders& encoders);
bool dropOff(float extra_offset, Motors& motors, Encoders& encoders);
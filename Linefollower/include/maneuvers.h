#pragma once

#include <Arduino.h>
#include "motors.h"
#include "encoders.h"

bool turnDegrees(float deg, Motors& motors, Encoders& encoders);
bool dropOff(Motors& motors, Encoders& encoders);
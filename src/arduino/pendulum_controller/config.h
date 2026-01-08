#pragma once

#include <Arduino.h>
#include "generated_config.h"

// Derived constants from generated_config.h
constexpr float MOTOR_COUNTS_PER_REV = MOTOR_COUNTS_PER_REV_4X / MOTOR_DECODE_FACTOR;
constexpr float SHAFT_DECODE_FACTOR = SHAFT_USE_CHANGE ? 2.0f : 1.0f;
constexpr float SHAFT_COUNTS_PER_REV = SHAFT_PPR * SHAFT_DECODE_FACTOR;

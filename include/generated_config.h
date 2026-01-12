#pragma once

// Auto-generated from config/pendulum_config.yaml. Do not edit by hand.

// Pins
constexpr uint8_t RPWM_PIN = 9;
constexpr uint8_t LPWM_PIN = 10;
constexpr uint8_t R_EN_PIN = 7;
constexpr uint8_t L_EN_PIN = 8;
constexpr uint8_t AM_PIN = 2;
constexpr uint8_t BM_PIN = 4;
constexpr uint8_t AS_PIN = 3;
constexpr uint8_t BS_PIN = 5;

// Encoder
constexpr float MOTOR_COUNTS_PER_REV_4X = 1717.9f;
constexpr float MOTOR_DECODE_FACTOR = 4.0f;
constexpr bool SHAFT_USE_CHANGE = false;
constexpr float SHAFT_PPR = 360.0f;

// Estimator
constexpr unsigned long EST_PERIOD_MS = 5UL;
constexpr float P_VEL_ALPHA = 0.25f;
constexpr float M_VEL_ALPHA = 0.25f;

// Control (swing/stabilize)
constexpr float P_VEL_DEADBAND = 1.0f;
constexpr int PWM_MAX_SWING = 120;
constexpr float THETA_STAB_RAD = 0.35f;
constexpr float KP = 60.0f;
constexpr float KD = 8.0f;
constexpr int PWM_MAX_STAB = 120;

// Motor
constexpr int PWM_LIMIT = 120;
constexpr int PWM_STEP_PER_CTRL = 8;
constexpr unsigned long DIR_CHANGE_ZERO_HOLD_MS = 30UL;

// Control loop
constexpr unsigned long CTRL_PERIOD_MS = 20UL;

// Serial
constexpr unsigned long BAUDRATE = 115200UL;

// Safety
constexpr float FULL_TURN_THRESHOLD_FRACTION = 0.9f;

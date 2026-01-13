#pragma once

#include <Arduino.h>

extern volatile long g_m_count;
extern volatile long g_p_count;

extern long p_zeroOffset;
extern long p_calibRawCount;

extern float p_angle;
extern float p_vel;
extern float m_vel;

extern unsigned long lastEstMs;
extern long lastPCountSnap;
extern long lastMCountSnap;

extern int p_vel_sign_latched;

extern int u_applied;
extern unsigned long dirChangeHoldUntilMs;

extern unsigned long lastCtrlMs;

extern bool estop;
extern bool paused;
extern bool fullTurnTripped;

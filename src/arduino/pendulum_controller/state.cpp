#include "state.h"

volatile long g_m_count = 0;
volatile long g_p_count = 0;

long p_zeroOffset = 0;
long p_calibRawCount = 0;

float p_angle = 0.0f;
float p_vel = 0.0f;
float m_vel = 0.0f;

unsigned long lastEstMs = 0;
long lastPCountSnap = 0;
long lastMCountSnap = 0;

int p_vel_sign_latched = 0;

int u_applied = 0;
unsigned long dirChangeHoldUntilMs = 0;

unsigned long lastCtrlMs = 0;

bool estop = false;
bool paused = true;
bool fullTurnTripped = false;

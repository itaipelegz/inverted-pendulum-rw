#pragma once

void allStopPWM();
void enableDriver();
void disableDriver();
void setMotorU(int u);
void applyMotorWithRamp(int u_target);

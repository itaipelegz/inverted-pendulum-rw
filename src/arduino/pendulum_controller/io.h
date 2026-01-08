#pragma once

long getMotorCount();
long getPendulumRawCount();
long getPendulumCount();

void motorA_isr();
void pendulumA_isr();

void handleSerial();
void printHelp();
void printSnapshot(const char* tag = "SNAP");
void setZeroAtUpUsingDownCalibration();

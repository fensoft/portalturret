#pragma once
extern int32_t smoothX;
extern int32_t smoothY;
extern int32_t smoothZ;
extern const int measurements;
extern bool accelerometerBuffered;

void setupAccelerometer();
void updateAccelerometer();

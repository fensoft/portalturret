#pragma once

#include "PortalTypes.h"

bool isOpen();
bool isDetectingMotion();
bool isPlayingAudio();
void setState(TurretState nextState);
void setManualState(ManualState nextState);
void manualRotation(unsigned long deltaTime);
void stateBehaviour();
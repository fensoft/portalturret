#include <Arduino.h>
#include "PortalServo.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "config.h"
#include "StateMachine.hpp"
#include "Accelerometer.h"

extern TurretMode currentTurretMode;
extern bool isDetectingMotionCached;
extern unsigned long lastMotionCheck;
extern TurretState currentState;
extern unsigned long previousTime;
extern bool wingsOpen;

// Global variables
bool fullyOpened = false;
unsigned long stateStartTime = 0;
ManualState currentManualState = ManualState::Idle;
unsigned long lastMovementTime = 0;

//Routines
#include <AceRoutine.h>
using namespace ace_routine;
#include "Routines.h"

bool isOpen() {
  return digitalRead(WING_SWITCH) == HIGH;
}

//For some reason we need to cache this value, as checking it every loop causes the webserver to freeze.
//https://github.com/me-no-dev/ESPAsyncWebServer/issues/944
bool isDetectingMotionCached = false;
unsigned long lastMotionCheck = 0;
bool isDetectingMotion() {
  unsigned long curMillis = millis();
  if (curMillis > lastMotionCheck + 50) {
    isDetectingMotionCached = analogRead(PID) > 512;
    lastMotionCheck = curMillis;
  }
  return isDetectingMotionCached;
}

void setState(TurretState nextState) {

  //Stop the Wing Servos just in case;
  wingServo.write(STATIONARY_ANGLE);
  
  if (currentTurretMode == TurretMode::Automatic) {
    switch (nextState) {
      case TurretState::Activated:
        activatedRoutine.reset();
        break;
      case TurretState::Engaging:
        engagingRoutine.reset();
        break;
      case TurretState::Searching:
        searchingRoutine.reset();
        break;
      case TurretState::TargetLost:
        targetLostRoutine.reset();
        break;
      case TurretState::PickedUp:
        pickedUpRoutine.reset();
        break;
      case TurretState::Shutdown:
        shutdownRoutine.reset();
        break;
      case TurretState::ManualEngaging:
        manualEngagingRoutine.reset();
        break;
      case TurretState::Rebooting:
        rebootRoutine.reset();
        break;
    }
    stateStartTime = millis();
    currentState = nextState;
  }
}

void setManualState(ManualState nextState) {

  //Stop the Wing Servos just in case;
  wingServo.write(STATIONARY_ANGLE);

  
  if (currentTurretMode == TurretMode::Manual) {
    switch (nextState) {
      case ManualState::Opening:
        openWingsRoutine.reset();
        break;
      case ManualState::Closing:
        closeWingsRoutine.reset();
        break;
      case ManualState::Firing:
        manualEngagingRoutine.reset();
        break;
    }
    currentManualState = nextState;
  }
}

void manualRotation(unsigned long deltaTime) {
  manualMovementRoutine.runCoroutine();
}
void stateBehaviour() {

  unsigned long deltaTime = millis() - previousTime;
  previousTime = millis();

  if (currentTurretMode == TurretMode::Manual) {
    switch (currentManualState) {
      case ManualState::Idle:
        manualRotation(deltaTime);
        break;
      case ManualState::Opening:
        openWingsRoutine.runCoroutine();
        if (openWingsRoutine.isDone()) {
          setManualState(ManualState::Idle);
        }
        break;
      case ManualState::Closing:
        closeWingsRoutine.runCoroutine();
        if (closeWingsRoutine.isDone()) {
          setManualState(ManualState::Idle);
        }
        break;
      case ManualState::Firing:
        manualRotation(deltaTime);
        manualEngagingRoutine.runCoroutine();
        if (manualEngagingRoutine.isDone()) {
          setManualState(ManualState::Idle);
        }
        break;
    }
  }
  if (currentTurretMode == TurretMode::Automatic) {
    
    bool motionDetected = isDetectingMotion();
    float zMovement = (smoothZ / measurements * SENSORS_GRAVITY_STANDARD * ADXL345_MG2G_MULTIPLIER);
    bool pickedUp = accelerometerBuffered && (zMovement < GFORCE_PICKED_UP_MIN || zMovement > GFORCE_PICKED_UP_MAX);
    bool movedAround = accelerometerBuffered && (zMovement < GFORCE_STEADY_MIN || zMovement > GFORCE_STEADY_MAX);
    bool onItsSide = accelerometerBuffered && (zMovement < TIPPED_OVER_Z_TRESHOLD);
    
    if (movedAround) {
      lastMovementTime = millis();
    }

    if (pickedUp && currentState != TurretState::PickedUp && currentState != TurretState::Shutdown && currentState != TurretState::Rebooting) {
      setState(TurretState::PickedUp);
    }
    switch (currentState) {
      case TurretState::Idle:
        if (motionDetected) {
          setState(TurretState::Activated);
        }
        break;
      case TurretState::Activated:
        activatedRoutine.runCoroutine();
        if (activatedRoutine.isDone()) {
          if (motionDetected) {
            setState(TurretState::Engaging);
          } else {
            setState(TurretState::Searching);
          }
        }
        break;
      case TurretState::Searching:
        searchingRoutine.runCoroutine();
        if (millis() > stateStartTime + 10000) {
          setState(TurretState::TargetLost);
        }
        if (motionDetected && millis() > stateStartTime + 100) {
          setState(TurretState::Engaging);
        }
        break;
      case TurretState::TargetLost:
        targetLostRoutine.runCoroutine();
        if (targetLostRoutine.isDone()) {
          setState(TurretState::Idle);
        }
        break;
      case TurretState::Engaging:
        engagingRoutine.runCoroutine();
        if (engagingRoutine.isDone()) {
          if (wingsOpen) {
            setState(TurretState::Searching);
          } else {
            setState(TurretState::Idle);
          }
        }
        break;
      case TurretState::PickedUp:
        pickedUpRoutine.runCoroutine();
        if (onItsSide) {
          setState(TurretState::Shutdown);
        }
        else if (!movedAround && millis() > lastMovementTime + 5000) {
          setState(TurretState::Activated);
        }
        break;
      case TurretState::Shutdown:
        shutdownRoutine.runCoroutine();
        if (shutdownRoutine.isDone() && !onItsSide) {
          setState(TurretState::Rebooting);
        }
        break;
      case TurretState::Rebooting:
        rebootRoutine.runCoroutine();
        if (rebootRoutine.isDone()) {
          setState(TurretState::Idle);
        }
        break;
    }
  }
}
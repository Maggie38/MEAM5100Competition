#include "autonomousattack.h"
#include "driver.h"
#include "motor.h"
#include "tof.h"
#include "vive.h"
#include "servo_ctrl.h"
#include <Arduino.h>

static int safeFront() {
  int v = (int)TOF3;
  return (v == 0) ? 255 : v;
}
static int safeRight() {
  int v = (int)TOF1;
  return (v == 0) ? 255 : v;
}
static int safeLeft() {
  int v = (int)TOF2;
  return (v == 0) ? 255 : v;
}

enum AutoLowStep {
  AL_WF_APPROACH = 0,  // wall follow left until right TOF < 15
  AL_TURN_RIGHT,        // pivot right until left encoder turned 550 counts
  AL_DONE
};

static AutoLowStep alStep    = AL_DONE;
static long        alTurnStart = 0;

void startAutoLow() {
  alStep     = AL_WF_APPROACH;
  alTurnStart = 0;
  resetWallFollow();
  resetPController();
  driveState = DS_AUTO_LOW;
}

void autoLowUpdate() {
  int rightDist = safeRight();

  noInterrupts();
  long currentL = encoderCount[0];
  interrupts();

  switch (alStep) {
    case AL_WF_APPROACH:
      wallFollowLeft();
      if (rightDist < 15) {
        motorsStop();
        resetWallFollow();
        alTurnStart = 0;
        alStep = AL_TURN_RIGHT;
      }
      break;

    case AL_TURN_RIGHT:
      if (alTurnStart == 0) {
        noInterrupts();
        alTurnStart = encoderCount[0];
        interrupts();
      }
      motor(1, 0, 0);
      motor(0, +1, 120);
      if (abs(currentL - alTurnStart) >= 5600) {
        motorsStop();
        alStep = AL_DONE;
        driveState = DS_STOP;
      }
      break;

    case AL_DONE:
    default:
      motorsStop();
      driveState = DS_STOP;
      break;
  }
}

constexpr int NEXUS_MAX_STRIKES = 4;

static long stateStartTime = 0;
static long stallCheckTimer = 0;
static long lastKnownCountL = 0;
static long stallStartTime = 0;

enum AutoNexusStep {
  AN_WF_TO_WALL = 0, // Wall follow left until stall/contact
  AN_BACK_UP_TOF,    // Back up until front TOF is 10cm
  AN_PIVOT_RIGHT,    // Pivot until encoder hits 2500
  AN_APPROACH_HIT,   // Wall follow until hit button (stall)
  AN_BACK_UP_STALL,  // Back up until stall
  AN_DONE
};

static AutoNexusStep anStep = AN_DONE;
static int anStrikes = 0; 
static long anEncoderStart = 0;

void transitionTo(AutoNexusStep nextStep) {
  anStep = nextStep;
  stateStartTime = millis();
  stallCheckTimer = 0;
  stallStartTime = 0;
  noInterrupts();
  lastKnownCountL = encoderCount[0];
  interrupts();
}

void startAutoNexus() {
  anStrikes = 0;
  resetPController();
  driveState = DS_AUTO_NEXUS;
  transitionTo(AN_WF_TO_WALL);
}

void autoNexusUpdate() {
  int frontDist = safeFront();
  
  noInterrupts();
  long currentL = encoderCount[0];
  interrupts();

  switch (anStep) {
    // 1. Wall follow left until stall
    case AN_WF_TO_WALL:
      wallFollowLeftSpeed(1500);
      if (isStalled()) {
        motorsStop();
        transitionTo(AN_BACK_UP_TOF);
      }
      break;

    // 2. Back up slowly until front TOF reads 10 cm
    case AN_BACK_UP_TOF:
      motor(0, -1, 80);
      motor(1, -1, 80);
      if (frontDist >= 100) { 
        motorsStop();
        anEncoderStart = currentL;
        transitionTo(AN_PIVOT_RIGHT);
      }
      break;

    // 3. Right wheel reverse, left wheel forward until encoder count 2500
    case AN_PIVOT_RIGHT:
      motor(0, +1, 100); 
      motor(1, -1, 100); 
      if (abs(currentL - anEncoderStart) >= 2500) {
        motorsStop();
        anStrikes = 0;
        transitionTo(AN_APPROACH_HIT);
      }
      break;

    // 4. Wall follow left until hit button (stall)
    case AN_APPROACH_HIT:
      wallFollowLeftSpeed(1200);
      if (isStalled()) { 
        motorsStop();
        transitionTo(AN_BACK_UP_STALL);
      }
      break;

    // 5. Back up until stall
    case AN_BACK_UP_STALL:
      motor(0, -1, 120);
      motor(1, -1, 120);
      
      if (isStalled()) { 
        motorsStop();
        anStrikes++;
        if (anStrikes < 4) {
          transitionTo(AN_APPROACH_HIT);
        } else {
          transitionTo(AN_DONE);
        }
      }
      break;

    case AN_DONE:
    default:
      motorsStop();
      driveState = DS_STOP;
      break;
  }
}

enum AutoHighStep {
  AH_RAMP_UP = 0,    // wall-follow right up the ramp at speed 1500 until right encoder >= 1600
  AH_TURN_RIGHT,     // pivot right (left motor fwd 70) until left encoder >= 300
  AH_DONE
};

static AutoHighStep ahStep      = AH_DONE;
static long         ahRampStart = 0;
static long         ahTurnStart = 0;

void startAutoHigh() {
  ahStep      = AH_RAMP_UP;
  ahRampStart = 0;
  ahTurnStart = 0;
  resetPController();
  resetWallFollow();
  driveState = DS_AUTO_HIGH;
}

void autoHighUpdate() {
  noInterrupts();
  long currentL = encoderCount[0];
  long currentR = encoderCount[1];
  interrupts();

  switch (ahStep) {
    case AH_RAMP_UP:
      if (ahRampStart == 0) {
        noInterrupts();
        ahRampStart = encoderCount[1];
        interrupts();
        currentR = ahRampStart;
      }
      wallFollowRightSpeed(1500);
      if (abs(currentR - ahRampStart) >= 14500) {
        motorsStop();
        resetWallFollow();
        ahTurnStart = 0;
        ahStep = AH_TURN_RIGHT;
      }
      break;

    case AH_TURN_RIGHT:
      if (ahTurnStart == 0) {
        noInterrupts();
        ahTurnStart = encoderCount[0];
        interrupts();
        currentL = ahTurnStart;
      }
      motor(0, +1, 100);  // left motor forward -> pivots right
      motor(1,  0,  0);  // right motor stopped
      if (abs(currentL - ahTurnStart) >= 3000) {
        motorsStop();
        ahStep = AH_DONE;
        driveState = DS_STOP;
      }
      break;

    case AH_DONE:
    default:
      motorsStop();
      driveState = DS_STOP;
      break;
  }
}
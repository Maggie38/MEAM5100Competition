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

void resetAutoLow() {
  alStep     = AL_DONE;
  alTurnStart = 0;
  resetWallFollow();
  motorsStop();
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

enum AutoNexusStep {
  AN_APPROACH = 0,   // drive toward nexus
  AN_STRIKE,         // fire servo
  AN_RETREAT,        // back away before next strike
  AN_DONE
};

static AutoNexusStep anStep    = AN_DONE;
static int           anStrikes = 0;       // counts strikes delivered (max 4)

constexpr int NEXUS_MAX_STRIKES = 4;

void startAutoNexus() {
  anStep    = AN_APPROACH;
  anStrikes = 0;
  resetPController();
  driveState = DS_AUTO_NEXUS;
}

void resetAutoNexus() {
  anStep    = AN_DONE;
  anStrikes = 0;
  motorsStop();
}

void autoNexusUpdate() {
  
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

void resetAutoHigh() {
  ahStep = AH_DONE;
  motorsStop();
  resetWallFollow();
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
      if (abs(currentR - ahRampStart) >= 10000) {
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
      motor(0, +1, 70);  // left motor forward -> pivots right
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
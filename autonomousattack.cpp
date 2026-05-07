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
  AL_WF_APPROACH = 0,  
  AL_TURN_RIGHT,   
  AL_FORWARD,
  AL_DONE
};

static AutoLowStep alStep    = AL_DONE;
static long        alTurnStart = 0;
static long         alForwardStart = 0;

static unsigned long alStartTime = 0; // NEW: Track when the state started

void startAutoLow() {
  alStep     = AL_WF_APPROACH;
  alTurnStart = 0;
  alForwardStart = 0;
  alStartTime = millis(); // NEW: Capture start time
  resetWallFollow();
  resetPController();
  driveState = DS_AUTO_LOW;
}

void autoLowUpdate() {
  int rightDist = safeRight();
  unsigned long now = millis(); // NEW: Get current time

  noInterrupts();
  long currentL = encoderCount[0];
  interrupts();

  switch (alStep) {
    case AL_WF_APPROACH:
      wallFollowLeftSpeed(350);
      
      // NEW: Only check right distance if 500ms has elapsed
      if (now - alStartTime > 500) { 
        if (rightDist < 15) {
          motorsStop();
          resetWallFollow();
          alTurnStart = 0;
          alStep = AL_TURN_RIGHT;
        }
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
      if (abs(currentL - alTurnStart) >= 4000) {
        motorsStop();
        alStep = AL_FORWARD;
      }
      break;
    
    case AL_FORWARD:
      if (alForwardStart == 0) {
        noInterrupts();
        alForwardStart = encoderCount[0];
        interrupts();
        currentL = alForwardStart;
      }
      motor(0, +1, 100);  // left motor forward
      motor(1,  +1,  100);  // right motor forward
      if (abs(currentL - alForwardStart) >= 500) {
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
constexpr int NEXUS_MAX_STRIKES = 8;

// Tune this on the robot. This is how far the robot wall-follows left
// before starting the Nexus alignment turns. It uses encoder counts instead
// of the front TOF, so increase it to drive farther before turning.
constexpr long NEXUS_WALL_FOLLOW_COUNTS = 17000;

// Still use front TOF only for the backup after each Nexus hit.
// If your TOF library reports mm, use 250 instead of 25.
constexpr int NEXUS_BACKUP_STOP_FRONT = 10;

// Tune these until each single-wheel pivot gives about 90 degrees.
constexpr long NEXUS_LEFT_WHEEL_90_COUNTS  = 1300;
constexpr long NEXUS_RIGHT_WHEEL_90_COUNTS = 1300;

// Use the already-tuned wallFollowLeft() for the initial approach.
// Do not force a custom low speed here; on this robot the speed-based helper
// may be too slow or may not match the tuned left-wall-follow behavior.
constexpr int NEXUS_TURN_SPEED        = 100;
constexpr int NEXUS_ATTACK_SPEED      = 75;
constexpr int NEXUS_BACKUP_SPEED      = 60;

// Prevent false-positive stalls immediately after a state starts.
constexpr unsigned long NEXUS_STALL_ARM_MS       = 300;
constexpr unsigned long NEXUS_ATTACK_TIMEOUT_MS  = 2500;
constexpr unsigned long NEXUS_BACKUP_TIMEOUT_MS  = 2500;
constexpr unsigned long NEXUS_TURN_TIMEOUT_MS    = 4000;

static unsigned long stateStartTime = 0;

// New sequence:
// 1. Wall follow left for a tunable encoder distance.
// 2. Turn the left wheel only until about 90 degrees.
// 3. Turn the right wheel only until about 90 degrees to align with nexus button.
// 4. Drive forward until stall/contact.
// 5. Back up until the FRONT TOF sees 25.
// 6. Repeat the hit/back-up cycle 4 times.
enum AutoNexusStep {
  AN_WF_FOR_ENCODER_DISTANCE = 0,
  AN_LEFT_WHEEL_90,
  AN_RIGHT_WHEEL_90,
  AN_ATTACK_FORWARD,
  AN_BACK_UP_TO_FRONT_25,
  AN_DONE
};

static AutoNexusStep anStep = AN_DONE;
static int  anStrikes = 0;
static long anEncoderStartL = 0;
static long anEncoderStartR = 0;

static void transitionTo(AutoNexusStep nextStep) {
  motorsStop();
  anStep = nextStep;
  stateStartTime = millis();

  noInterrupts();
  anEncoderStartL = encoderCount[0];
  anEncoderStartR = encoderCount[1];
  interrupts();
}

static bool stateHasRunFor(unsigned long ms) {
  return (millis() - stateStartTime) >= ms;
}

void startAutoNexus() {
  anStrikes = 0;
  anEncoderStartL = 0;
  anEncoderStartR = 0;
  resetPController();
  resetWallFollow();
  driveState = DS_AUTO_NEXUS;
  transitionTo(AN_WF_FOR_ENCODER_DISTANCE);
}

void autoNexusUpdate() {
  int frontDist = safeFront();

  noInterrupts();
  long currentL = encoderCount[0];
  long currentR = encoderCount[1];
  interrupts();

  switch (anStep) {
    case AN_WF_FOR_ENCODER_DISTANCE: {
      // Use the same left-wall-follow routine that already works in autoLow,
      // but exit using encoder travel instead of the front TOF.
      wallFollowLeftSpeed(350);

      long leftTravel  = labs(currentL - anEncoderStartL);
      long rightTravel = labs(currentR - anEncoderStartR);
      long avgTravel   = (leftTravel + rightTravel) / 2;

      if (avgTravel >= NEXUS_WALL_FOLLOW_COUNTS) {
        resetWallFollow();
        transitionTo(AN_LEFT_WHEEL_90);
      }
      break;
    }

    case AN_LEFT_WHEEL_90:
      // Left wheel only. Based on your existing convention, motor 0 is left.
      motor(0, +1, NEXUS_TURN_SPEED);
      motor(1,  0, 0);

      if (labs(currentL - anEncoderStartL) >= NEXUS_LEFT_WHEEL_90_COUNTS ||
          stateHasRunFor(NEXUS_TURN_TIMEOUT_MS)) {
        transitionTo(AN_RIGHT_WHEEL_90);
      }
      break;

    case AN_RIGHT_WHEEL_90:
      // Right wheel only. Based on your existing convention, motor 1 is right.
      motor(0,  1, 0);
      motor(1, +1, NEXUS_TURN_SPEED);

      if (labs(currentR - anEncoderStartR) >= NEXUS_RIGHT_WHEEL_90_COUNTS ||
          stateHasRunFor(NEXUS_TURN_TIMEOUT_MS)) {
        transitionTo(AN_ATTACK_FORWARD);
      }
      break;

    case AN_ATTACK_FORWARD:
      motor(0, +1, NEXUS_ATTACK_SPEED);
      motor(1, +1, NEXUS_ATTACK_SPEED);

      // Stall means we pushed into the nexus button.
      // Timeout keeps the robot from driving forever if stall detection misses.
      if ((stateHasRunFor(NEXUS_STALL_ARM_MS) && isStalled()) ||
          stateHasRunFor(NEXUS_ATTACK_TIMEOUT_MS)) {
        anStrikes++;
        transitionTo(AN_BACK_UP_TO_FRONT_25);
      }
      break;

    case AN_BACK_UP_TO_FRONT_25:
      motor(0, -1, NEXUS_BACKUP_SPEED);
      motor(1, -1, NEXUS_BACKUP_SPEED);

      // After the hit, backing up should make front distance increase.
      // Stop once the front TOF sees at least 25.
      if (frontDist >= NEXUS_BACKUP_STOP_FRONT ||
          stateHasRunFor(NEXUS_BACKUP_TIMEOUT_MS)) {
        if (anStrikes < NEXUS_MAX_STRIKES) {
          transitionTo(AN_ATTACK_FORWARD);
        } else {
          transitionTo(AN_DONE);
        }
      }
      break;

    case AN_DONE:
    default:
      motorsStop();
      resetWallFollow();
      driveState = DS_STOP;
      break;
  }
}

enum AutoHighStep {
  AH_RAMP_UP = 0,    // wall-follow right up the ramp at speed 1500 until right encoder >= 1600
  AH_TURN_RIGHT,     // pivot right (left motor fwd 70) until left encoder >= 300
  AH_FORWARD,        //drive straight to ensure button press
  AH_DONE
};

static AutoHighStep ahStep      = AH_DONE;
static long         ahRampStart = 0;
static long         ahTurnStart = 0;
static long         ahForwardStart = 0;

void startAutoHigh() {
  ahStep      = AH_RAMP_UP;
  ahRampStart = 0;
  ahTurnStart = 0;
  ahForwardStart = 0;
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
      if (abs(currentR - ahRampStart) >= 13000) {
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
      if (abs(currentL - ahTurnStart) >= 1600) {
        motorsStop();
        ahStep = AH_FORWARD;
      }
      break;
    case AH_FORWARD:
      if (ahForwardStart == 0) {
        noInterrupts();
        ahForwardStart = encoderCount[0];
        interrupts();
        currentL = ahForwardStart;
      }
      motor(0, +1, 100);  // left motor forward
      motor(1,  +1,  100);  // right motor forward
      if (abs(currentL - ahForwardStart) >= 500) {
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
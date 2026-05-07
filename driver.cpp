#include "driver.h"
#include "motor.h"
#include "tof.h"
#include "vive.h"
#include "autonomousattack.h"
#include <math.h>

// current drive state
volatile DriveState driveState = DS_STOP;

static long prevCount[2]  = {0, 0};
static long lastControlMS = 0;
volatile int pwmOutput[2]  = {PWM_MIN, PWM_MIN};

// target wall follow distance
static int targetTOF_right = -1;
static int targetTOF_left  = -1;

static int wfPrevErrorRight = 0;
static int wfPrevErrorLeft  = 0;

// Left wall-follow backup timing
static const unsigned long LEFT_WALL_FOLLOW_BACKUP_MS = 750;
static unsigned long leftWallFollowBackupStartMS = 0;

// Left wall-follow speed backup timing
static const unsigned long LEFT_WALL_FOLLOW_SPEED_BACKUP_MS = 750;
static unsigned long leftWallFollowSpeedBackupStartMS = 0;

// target vive locations
static float targetX = 0;
static float targetY = 0;

// initialize motor pwm to be min
void driverInit() {
  for (int m = 0; m < 2; m++) {
    pwmOutput[m] = PWM_MIN;
    prevCount[m] = 0;
  }
  lastControlMS = millis();
  targetTOF_right = -1;
  targetTOF_left  = -1;
}

// reset p controller
void resetPController() {
  for (int m = 0; m < 2; m++) {
    pwmOutput[m] = PWM_MIN;
    noInterrupts();
    prevCount[m] = encoderCount[m];
    interrupts();
  }
}

// reset latched wall-follow targets so the next call re-latches
void resetWallFollow() {
  targetTOF_right = -1;
  targetTOF_left  = -1;
  wfPrevErrorRight = 0;
  wfPrevErrorLeft  = 0;
  leftWallFollowBackupStartMS = 0;
}

// one step of p controller
void PController(int motorDir, int targetspeed) {
  // Read both encoder counts atomically
  noInterrupts();
  long countL = encoderCount[0];
  long countR = encoderCount[1];
  interrupts();

  // Calculate individual speeds
  long speedL = abs(countL - prevCount[0]);
  long speedR = abs(countR - prevCount[1]);
  prevCount[0] = countL;
  prevCount[1] = countR;

  // Keep both motors near targetspeed using average as the base reading
  long avgSpeed = (speedL + speedR) / 2;

  // Base correction: push average toward targetspeed
  long baseError      = targetspeed - avgSpeed;
  int  baseCorrection = (int)(KP * (float)baseError);

  // Differential correction: error > 0 means left is faster than right
  long diffError = speedL - speedR;
  int  diffCorr  = (int)(KP_DIFF * (float)diffError);

  // Left motor gets less PWM if it's faster, right gets more
  // If left is slower, left gets more PWM, right gets less
  pwmOutput[0] = constrain(pwmOutput[0] + baseCorrection - diffCorr, PWM_MIN, PWM_MAX);
  pwmOutput[1] = constrain(pwmOutput[1] + baseCorrection + diffCorr, PWM_MIN, PWM_MAX);

  motor(0, motorDir, pwmOutput[0]);
  motor(1, motorDir, pwmOutput[1]);
}

void PController(int motorDir) {
  PController(motorDir, TARGET_SPEED);
}

// update driver if health > 0
void driverUpdate(byte health) {
  long now = millis();
  if (now - lastControlMS < CONTROL_MS) return;
  lastControlMS = now;

  if (health == 0) return; 

  if      (driveState == DS_FORWARD)          PController(+1);
  else if (driveState == DS_BACKWARD)         PController(-1);
  else if (driveState == DS_WALL_FOLLOW_RIGHT) wallFollowRight();
  else if (driveState == DS_WALL_FOLLOW_LEFT)  wallFollowLeftSpeed(350);
  else if (driveState == DS_AUTO_CIRCUIT)      autoCircuitUpdate();
  else if (driveState == DS_AUTO_TARGET)       autoTargetUpdate();
  else if (driveState == DS_AUTO_LOW)   autoLowUpdate();
  else if (driveState == DS_AUTO_NEXUS) autoNexusUpdate();
  else if (driveState == DS_AUTO_HIGH)  autoHighUpdate();
}

/*
Wall following — target-distance PD controller
 
On first tick after entering wall-follow mode, set target_dist = current_dist
  error = current - target
  error > 0 - robot drifted away from wall - steer back towards wall
  error < 0 - robot drifted into wall      - steer away
 
  Right wall (TOF1): error < 0 - more right motor
                     error > 0 - more left motor
*/
void wallFollowRight() {
  int front = (int)TOF3;
  if (front == 0) front = 255;
 
  if (front < 25) {
    motor(0, -1, 70);
    motor(1,  -1,  40);
    return;
  }
 
  int current = (int)TOF1;
  if (current == 0) current = 255; 
 
  // Latch target distance on first call after entering this mode
  if (targetTOF_right < 0) targetTOF_right = current;
 
  // error > 0: drifted away from wall - boost left motor
  // error < 0: too close to wall - boost right motor
  int error    = current - targetTOF_right;
  int dError   = error - wfPrevErrorRight;
  wfPrevErrorRight = error;
  int delta    = constrain((int)(KP_WF * error + KD_WF * dError), -WF_DELTA_MAX, WF_DELTA_MAX);
 
  pwmOutput[0] = constrain(WF_BASE_PWM + delta, PWM_MIN, PWM_MAX);
  pwmOutput[1] = constrain(WF_BASE_PWM - delta, PWM_MIN, PWM_MAX);
 
  // update motor calls
  motor(0, +1, pwmOutput[0]);
  motor(1, +1, pwmOutput[1]);
}
 
void wallFollowLeft() {
  unsigned long now = millis();

  int front = (int)TOF3;
  if (front == 0) front = 255;

  // If the front sensor sees an obstacle, start a timed backup
  if (front < 15 && leftWallFollowBackupStartMS == 0) {
    leftWallFollowBackupStartMS = now;
  }

  // While backup mode is active, keep backing up until the timer expires.
  if (leftWallFollowBackupStartMS != 0) {
    if (now - leftWallFollowBackupStartMS < LEFT_WALL_FOLLOW_BACKUP_MS) {
      motor(0, -1, 60);
      motor(1, -1, 90);
      return;
    }

    // Backup finished. Reset the timer and re-latch the left wall target
    leftWallFollowBackupStartMS = 0;
    targetTOF_left = -1;
    wfPrevErrorLeft = 0;
    motorsStop();
    return;
  }
 
  int current = (int)TOF2;
  if (current == 0) current = 255;
 
  // Latch target distance on first call after entering this mode
  if (targetTOF_left < 0) targetTOF_left = current;
 
  // error > 0: drifted away from wall - boost right motor
  // error < 0: too close to wall - boost left motor
  int error    = current - targetTOF_left;
  int dError   = error - wfPrevErrorLeft;
  wfPrevErrorLeft = error;
  int delta    = constrain((int)(KP_WF * error + KD_WF * dError), -WF_DELTA_MAX, WF_DELTA_MAX);
 
  pwmOutput[1] = constrain(WF_BASE_PWM + delta, PWM_MIN, PWM_MAX);
  pwmOutput[0] = constrain(WF_BASE_PWM - delta, PWM_MIN, PWM_MAX);
 
  // update motor calls
  motor(0, +1, pwmOutput[0]);
  motor(1, +1, pwmOutput[1]);
}

void wallFollowRightSpeed(long targetSpeed) {
  int current = (int)TOF1;
  if (current == 0) current = 255;

  if (targetTOF_right < 0) targetTOF_right = current;

  // Speed regulation (same as PController but single base)
  noInterrupts();
  long countL = encoderCount[0];
  long countR = encoderCount[1];
  interrupts();

  long speedL = abs(countL - prevCount[0]);
  long speedR = abs(countR - prevCount[1]);
  prevCount[0] = countL;
  prevCount[1] = countR;

  long avgSpeed = (speedL + speedR) / 2;
  long baseError = targetSpeed - avgSpeed;
  int baseCorrection = (int)(KP * (float)baseError);

  // Wall steering correction
  int error = current - targetTOF_right;
  int dError = error - wfPrevErrorRight;
  wfPrevErrorRight = error;
  int delta = constrain((int)(KP_WF * error + KD_WF * dError), -WF_DELTA_MAX, WF_DELTA_MAX);

  // Apply speed regulation to both, then wall correction on top
  pwmOutput[0] = constrain(pwmOutput[0] + baseCorrection, PWM_MIN, PWM_MAX+16);
  pwmOutput[1] = constrain(pwmOutput[1] + baseCorrection, PWM_MIN, PWM_MAX+16);

  pwmOutput[0] = constrain(pwmOutput[0] + delta, PWM_MIN, PWM_MAX+16);
  pwmOutput[1] = constrain(pwmOutput[1] - delta, PWM_MIN, PWM_MAX+16);

  motor(0, +1, pwmOutput[0]);
  motor(1, +1, pwmOutput[1]);
}

void wallFollowLeftSpeed(long targetSpeed) {
  unsigned long now = millis();
 
  int front = (int)TOF3;
  if (front == 0) front = 255;
 
  // If the front sensor sees an obstacle, start a timed backup
  if ((front < 15 || isStalled()) && leftWallFollowSpeedBackupStartMS == 0) {
    leftWallFollowSpeedBackupStartMS = now;
  }
 
  // While backup mode is active, keep backing up until the timer expires.
  if (leftWallFollowSpeedBackupStartMS != 0) {
    if (now - leftWallFollowSpeedBackupStartMS < LEFT_WALL_FOLLOW_SPEED_BACKUP_MS) {
      // Bias right motor faster so robot curves right, away from front wall,
      // while staying set up to re-acquire the left wall
      motor(0, -1, 60);
      motor(1, -1, 90);
      return;
    }
 
    // Backup finished. Reset timer and re-latch the left wall target
    leftWallFollowSpeedBackupStartMS = 0;
    targetTOF_left = -1;
    wfPrevErrorLeft = 0;
    motorsStop();
    return;
  }
 
  // Use TOF2 for the left side
  int current = (int)TOF2;
  if (current == 0) current = 255;
 
  // Initialize target if not already set
  if (targetTOF_left < 0) targetTOF_left = current;
 
  // 1. Speed regulation (P-Controller for base speed)
  noInterrupts();
  long countL = encoderCount[0];
  long countR = encoderCount[1];
  interrupts();
 
  long speedL = abs(countL - prevCount[0]);
  long speedR = abs(countR - prevCount[1]);
  prevCount[0] = countL;
  prevCount[1] = countR;
 
  long avgSpeed = (speedL + speedR) / 2;
  long baseError = targetSpeed - avgSpeed;
  int baseCorrection = (int)(KP * (float)baseError);
 
  // 2. Wall steering correction (PD Logic)
  int error = current - targetTOF_left;
  int dError = error - wfPrevErrorLeft;
  wfPrevErrorLeft = error;
  
  // Calculate steering delta
  int delta = constrain((int)(KP_WF * error + KD_WF * dError), -WF_DELTA_MAX, WF_DELTA_MAX);
 
  // 3. Apply base speed correction to both motors
  pwmOutput[0] = constrain(pwmOutput[0] + baseCorrection, PWM_MIN, PWM_MAX+10);
  pwmOutput[1] = constrain(pwmOutput[1] + baseCorrection, PWM_MIN, PWM_MAX+10);
 
  // 4. Apply steering correction
  pwmOutput[0] = constrain(pwmOutput[0] - delta, PWM_MIN, PWM_MAX+10);
  pwmOutput[1] = constrain(pwmOutput[1] + delta, PWM_MIN, PWM_MAX+10);
 
  motor(0, +1, pwmOutput[0]);
  motor(1, +1, pwmOutput[1]);
}

static long acTurnStartPos = 0;

enum ACStep {
  AC_WF_1_HIT = 0, // 1. Right wall follow 1500 until hit
  AC_BACK_1,       // 2. Back up until -1200
  AC_TURN_1,       // 3. Left wheel reverse until TOF > 60
  AC_STR_1_HIT,    // 4. Drive straight until hit
  AC_BACK_2,       // 5. Back up until -1200
  AC_TURN_2,       // 6. Left wheel reverse until TOF > 30
  AC_WF_2_HIT,     // 7. Right wall follow until hit
  AC_BACK_3,       // 8. Back up until -1500
  AC_TURN_3,       // 9. Left rev + Right fwd until TOF > 60
  AC_STR_2_HIT,    // 10. Drive straight until hit
  AC_DONE
};

static ACStep acStep = AC_WF_1_HIT;
static long startEncoderL = 0;
static long startEncoderR = 0;
static long lastEncoderCheckMS = 0;
static long lastKnownCountL = 0;

void resetAutoCircuit() {
  acStep = AC_WF_1_HIT;
  acTurnStartPos = 0;
  resetWallFollow();
  resetPController();
  Serial.println("reset auto circuit");
}

static long stateStartTime = 0;      // When the current step started
static long stallCheckTimer = 0;     // Timer for the 100ms window
static long stallStartTime = 0;      // How long we have been "stuck"

static void nextStep() {
  motorsStop();
  resetWallFollow();
  acStep = (ACStep)((int)acStep + 1);
  if (acStep >= AC_DONE) acStep = AC_WF_1_HIT;
  
  stateStartTime = millis(); 
  stallStartTime = 0;
  stallCheckTimer = millis();
  
  noInterrupts();
  startEncoderL = encoderCount[0];
  lastKnownCountL = startEncoderL;
  interrupts();
}

bool isStalled() {
  long now = millis();
  
  // Grace period: Ignore stalls for the first 300ms of a new state
  if (now - stateStartTime < 300) return false;

  // Sample encoder every 100ms
  if (now - stallCheckTimer > 100) {
    noInterrupts();
    long currentL = encoderCount[0];
    interrupts();
    
    // Check if we moved less than 5 ticks
    if (abs(currentL - lastKnownCountL) < 5) {
      if (stallStartTime == 0) stallStartTime = now;
    } else {
      stallStartTime = 0;
    }
    
    lastKnownCountL = currentL;
    stallCheckTimer = now;
  }

  if (stallStartTime != 0 && (now - stallStartTime > 200)) {
    return true;
  }

  return false;
}

void autoCircuitUpdate() {
  int front = (int)TOF3;
  if (front == 0) front = 255;

  noInterrupts();
  long currentL = encoderCount[0];
  long currentR = encoderCount[1];
  interrupts();

  switch (acStep) {
    
    // 1. Right wall follow until hit
    case AC_WF_1_HIT:
    case AC_WF_2_HIT:
      wallFollowRightSpeed(1500);
      if (isStalled()) {
        startEncoderL = currentL; 
        nextStep();
      }
      break;

    // 2. Back up until -1200 encoder count
    case AC_BACK_1:
      motor(0, -1, 100);
      motor(1, -1, 100);
      if (abs(currentL - startEncoderL) >= 1200) {
        nextStep();
      }
      break;

    // 3. Left wheel reverse and right wheel forward until front TOF > 60
    case AC_TURN_1:
    case AC_TURN_3:
      motor(0, -1, 80);
      motor(1, 1, 80);
      if (front > 60) nextStep();
      break;

    // 4. Drive straight until hit wall
    case AC_STR_1_HIT:
      motor(0, +1, 100);
      motor(1, +1, 100);
      if (isStalled()) {
        startEncoderL = currentL;
        nextStep();
      }
      break;

    // 5. Back up until -1200
    case AC_BACK_2:
      motor(0, -1, 100);
      motor(1, -1, 100);
      if (abs(currentL - startEncoderL) >= 1200) nextStep();
      break;

    // 6. Left wheel reverse (turn left) until front TOF > 30
    case AC_TURN_2:
      motor(0, -1, 100);
      motor(1, 0, 0);
      if (front > 30) nextStep();
      break;

    // 7. Back up until -1500
    case AC_BACK_3:
      motor(0, -1, 100);
      motor(1, -1, 100);
      if (abs(currentL - startEncoderL) >= 1500) nextStep();
      break;

    // 9. Drive straight until hit
    case AC_STR_2_HIT:
      motor(0, +1, 100);
      motor(1, +1, 100);
      if (isStalled()) nextStep();
      break;

    default:
      motorsStop();
      break;
  }
}

// autonomous vive cover logic
// New sequence:
//   1. AT_DRIVE_X  — drive straight slowly until robotX is within 100 of targetX
//   2. AT_TURN_90  — pivot right (left fwd, right rev) by AC_TURN90_TICKS encoder counts
//   3. AT_DRIVE_Y  — drive straight slowly until robotY is within 100 of targetY
enum AutoTargetStep {
  AT_DRIVE_X,
  AT_TURN_90,
  AT_DRIVE_Y,
  AT_DONE
};
 
static AutoTargetStep autoTargetStep = AT_DONE;
 
constexpr float VIVE_TOL   = 100.0f;  // coordinate tolerance (Vive units)
constexpr int   AT_TURN_PWM  = 90;    // pivot turn PWM
 
static long atTurnStartL = 0;
static long atTurnStartR = 0;

static int consecutiveViveHits = 0; 

void startAutoTarget(float tx, float ty) {
  targetX = tx;
  targetY = ty;
  autoTargetStep = AT_DRIVE_X;
  consecutiveViveHits = 0; // Reset counter for the new task
  resetPController();
  driveState = DS_AUTO_TARGET;
}

void autoTargetUpdate() {
  noInterrupts();
  long currentL = encoderCount[0];
  long currentR = encoderCount[1];
  interrupts();
 
  switch (autoTargetStep) {
 
    case AT_DRIVE_X:
      // Drive straight slowly until X coordinate is close enough 5 times in a row
      if (fabsf(robotX - targetX) <= VIVE_TOL) {
        consecutiveViveHits++;
      } else {
        consecutiveViveHits = 0; // Reset if we get a stray/out-of-bounds reading
      }

      if (consecutiveViveHits >= 3) {
        motorsStop();
        consecutiveViveHits = 0; // Reset counter for the Y phase
        
        // Latch encoder positions for the upcoming turn
        noInterrupts();
        atTurnStartL = encoderCount[0];
        atTurnStartR = encoderCount[1];
        interrupts();
        
        autoTargetStep = AT_TURN_90;
        return;
      }
      wallFollowLeftSpeed(350);
      break;
 
    case AT_TURN_90:
      // Pivot right: left motor forward, right motor reverse
      motor(0, +1, AT_TURN_PWM);
      motor(1, -1, AT_TURN_PWM);
      if (abs(currentL - atTurnStartL) >= AC_TURN90_TICKS &&
          abs(currentR - atTurnStartR) >= AC_TURN90_TICKS) {
        motorsStop();
        autoTargetStep = AT_DRIVE_Y;
      }
      break;
 
    case AT_DRIVE_Y:
      // Drive straight slowly until Y coordinate is close enough 5 times in a row
      if (fabsf(robotY - targetY) <= VIVE_TOL) {
        consecutiveViveHits++;
      } else {
        consecutiveViveHits = 0; // Reset if we get a stray reading
      }

      if (consecutiveViveHits >= 3) {
        motorsStop();
        consecutiveViveHits = 0;
        autoTargetStep = AT_DONE;
        driveState = DS_STOP;
        return;
      }
      PController(+1, 400);
      break;
 
    default:
      motorsStop();
      driveState = DS_STOP;
      break;
  }
}
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

// Left wall-follow backup timing.
// Change this value to tune how long the robot backs up when the front TOF sees an obstacle.
static const unsigned long LEFT_WALL_FOLLOW_BACKUP_MS = 750;
static unsigned long leftWallFollowBackupStartMS = 0;

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
void PController(int motorDir) {
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

  // Keep both motors near TARGET_SPEED using average as the base reading
  long avgSpeed = (speedL + speedR) / 2;

  // Base correction: push average toward TARGET_SPEED
  long baseError      = TARGET_SPEED - avgSpeed;
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

// update driver if health > 0
void driverUpdate(byte health) {
  long now = millis();
  if (now - lastControlMS < CONTROL_MS) return;
  lastControlMS = now;

  if (health == 0) return; 

  if      (driveState == DS_FORWARD)          PController(+1);
  else if (driveState == DS_BACKWARD)         PController(-1);
  else if (driveState == DS_WALL_FOLLOW_RIGHT) wallFollowRight();
  else if (driveState == DS_WALL_FOLLOW_LEFT)  wallFollowLeft();
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

  // If the front sensor sees an obstacle, start a timed backup.
  // This makes the backup last a fixed amount of time instead of depending
  // on how long the front TOF stays below the threshold.
  if (front < 20 && leftWallFollowBackupStartMS == 0) {
    leftWallFollowBackupStartMS = now;
  }

  // While backup mode is active, keep backing up until the timer expires.
  if (leftWallFollowBackupStartMS != 0) {
    if (now - leftWallFollowBackupStartMS < LEFT_WALL_FOLLOW_BACKUP_MS) {
      motor(0, -1, 40);
      motor(1, -1, 70);
      return;
    }

    // Backup finished. Reset the timer and re-latch the left wall target
    // because the robot's distance from the wall may have changed.
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
  // For Left Wall Follow:
  // If error is positive (too far), delta is positive.
  // We want to turn left: Decrease Left Motor (0), Increase Right Motor (1)
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
enum AutoTargetStep {
  AT_TURN,
  AT_DRIVE,
  AT_DONE
};

static AutoTargetStep autoTargetStep = AT_DONE;

constexpr float ANGLE_TOL_DEG = 8.0;     // acceptable heading error
constexpr float DIST_TOL = 150.0;        // Vive coordinate tolerance
constexpr int TURN_PWM = 100;
constexpr int DRIVE_PWM = 115;

static float angleWrapDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void startAutoTarget(float tx, float ty) {
  targetX = tx;
  targetY = ty;
  autoTargetStep = AT_TURN;
  resetPController();
  driveState = DS_AUTO_TARGET;
}

void autoTargetUpdate() {
  float dx = targetX - robotX;
  float dy = targetY - robotY;

  float targetAngle = atan2(dy, dx) * 180.0f / PI;
  float angleError = angleWrapDeg(targetAngle - robotTheta);
  float dist = sqrt(dx * dx + dy * dy);

  if (autoTargetStep == AT_TURN) {
    if (abs(angleError) <= ANGLE_TOL_DEG) {
      motorsStop();
      resetPController();
      autoTargetStep = AT_DRIVE;
      return;
    }

    if (angleError > 0) {
      // turn left
      motor(0, -1, TURN_PWM);
      motor(1, +1, TURN_PWM);
    } else {
      // turn right
      motor(0, +1, TURN_PWM);
      motor(1, -1, TURN_PWM);
    }
  }

  else if (autoTargetStep == AT_DRIVE) {
    if (dist <= DIST_TOL) {
      motorsStop();
      autoTargetStep = AT_DONE;
      driveState = DS_STOP;
      return;
    }

    int correction = constrain((int)(1.5f * angleError), -40, 40);

    int leftPWM  = constrain(DRIVE_PWM - correction, PWM_MIN, PWM_MAX);
    int rightPWM = constrain(DRIVE_PWM + correction, PWM_MIN, PWM_MAX);

    motor(0, +1, leftPWM);
    motor(1, +1, rightPWM);
  }

  else {
    motorsStop();
    driveState = DS_STOP;
  }
}

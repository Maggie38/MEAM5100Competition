#include "driver.h"
#include "motor.h"
#include "tof.h"
#include "vive.h"
#include <math.h>

// current drive state
volatile DriveState driveState = DS_STOP;

static long prevCount[2]  = {0, 0};
static long lastControlMS = 0;
int pwmOutput[2]  = {PWM_MIN, PWM_MIN};

// target wall follow distance
static int targetTOF_right = -1;
static int targetTOF_left  = -1;

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
}

/*
Wall following — target-distance P controller

On first tick after entering wall-follow mode, set target_dist = current_dist
  error = current - target
  error > 0 - robot drifted away from wall - steer back towards wall
  error < 0 - robot drifted into wall      - steer away

  Right wall (TOF1): error < 0 - more right motor
                     error > 0 - more left motor
*/
void wallFollowRight() {
  int current = (int)TOF1;
  if (current == 0) current = 255; 

  // Latch target distance on first call after entering this mode
  if (targetTOF_right < 0) targetTOF_right = current;

  // error > 0: drifted away from wall - boost left motor
  // error < 0: too close to wall - boost right motor
  int error    = current - targetTOF_right;
  int delta    = constrain((int)(KP_WF * error), -WF_DELTA_MAX, WF_DELTA_MAX);

  if (delta > 0)
    pwmOutput[0] = constrain(WF_BASE_PWM + delta, PWM_MIN, PWM_MAX);
  else
    pwmOutput[1] = constrain(WF_BASE_PWM - delta, PWM_MIN, PWM_MAX);

  // update motor calls
  motor(0, +1, pwmOutput[0]);
  motor(1, +1, pwmOutput[1]);
}

void wallFollowLeft() {
  int current = (int)TOF2;
  if (current == 0) current = 255;

  // Latch target distance on first call after entering this mode
  if (targetTOF_left < 0) targetTOF_left = current;

  // error > 0: drifted away from wall - boost right motor
  // error < 0: too close to wall - boost left motor
  int error    = current - targetTOF_left;
  int delta    = constrain((int)(KP_WF * error), -WF_DELTA_MAX, WF_DELTA_MAX);

  if (delta > 0)
    pwmOutput[1] = constrain(WF_BASE_PWM + delta, PWM_MIN, PWM_MAX);
  else
    pwmOutput[0] = constrain(WF_BASE_PWM - delta, PWM_MIN, PWM_MAX);

  // update motor calls
  motor(0, +1, pwmOutput[0]);
  motor(1, +1, pwmOutput[1]);
}

static long acTurnStartPos = 0;
static long acDriveStartCount = 0;

enum ACStep {
  AC_WF_START = 0,
  AC_TL_90,
  AC_WF_TO_TOWER,
  AC_WL_WEAVE,
  AC_STR_CLEAR,
  AC_WR_PASS,
  AC_STR_LONG,
  AC_WR_RETURN,
  AC_STR_BACK,
  AC_WL_ALIGN,
  AC_NUM_STEPS
};

static ACStep acStep = AC_WF_START;

void resetAutoCircuit() {
  acStep = AC_WF_START;
  acTurnStartPos = 0;
  resetWallFollow();
  resetPController();
}

// Start a Left Turn using encoders (Right motor moves forward)
static void startTurnLeftEncoder() {
  motor(0, 0, 0);                 // Left motor stop
  motor(1, +1, 110);      // Right motor forward
  noInterrupts();
  acTurnStartPos = encoderCount[1]; // Track right motor encoder
  interrupts();
}

// Start a Right Turn using encoders (Left motor moves forward)
static void startTurnRightEncoder() {
  motor(0, +1, 110);      // Left motor forward
  motor(1, 0, 0);                 // Right motor stop
  noInterrupts();
  acTurnStartPos = encoderCount[0]; // Track left motor encoder
  interrupts();
}

// Advance acStep and re-latch wall follow
static void nextStep() {
  motorsStop();

  // Clear all tracking variables for the next state
  acTurnStartPos = 0;
  acDriveStartCount = 0;
  resetWallFollow(); 

  // advance to next step (back to 0 if done)
  acStep = (ACStep)((int)acStep + 1);
  if (acStep >= AC_NUM_STEPS) acStep = AC_WF_START;
}

// execute current step of wall follow
void autoCircuitUpdate() {
  int front = (int)TOF3;
  if (front == 0) front = 255;

  noInterrupts();
  long currentL = encoderCount[0];
  long currentR = encoderCount[1];
  interrupts();

  switch (acStep) {
    // Right wall follow
    case AC_WF_START:
    case AC_WF_TO_TOWER:
      wallFollowRight();
      if (front <= AC_FRONT_WALL_CM) nextStep();
      break;

    // Turning left at corner
    case AC_TL_90:
      if (acTurnStartPos == 0) startTurnLeftEncoder();
      if (abs(currentR - acTurnStartPos) >= AC_TURN90_TICKS) nextStep();
      break;

    // Weave around towers
    case AC_WL_WEAVE:
    case AC_WL_ALIGN:
      if (acTurnStartPos == 0) startTurnLeftEncoder();
      if (abs(currentR - acTurnStartPos) >= AC_WEAVE_TICKS) nextStep();
      break;

    case AC_WR_PASS:
    case AC_WR_RETURN:
      if (acTurnStartPos == 0) startTurnRightEncoder();
      if (abs(currentL - acTurnStartPos) >= AC_WEAVE_TICKS) nextStep();
      break;

    // Straights around the towers
    case AC_STR_CLEAR:
    case AC_STR_LONG:
    case AC_STR_BACK:
      if (acDriveStartCount == 0) {
        resetPController();
        acDriveStartCount = (currentL + currentR) / 2;
      }
      PController(+1);
      
      long target = (acStep == AC_STR_LONG) ? AC_TICKS_30CM : AC_TICKS_20CM;
      if (abs(((currentL + currentR) / 2) - acDriveStartCount) >= target) {
        acDriveStartCount = 0;
        nextStep();
      }
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
#include "driver.h"
#include "motor.h"
#include "tof.h"

// current drive state
volatile DriveState driveState = DS_STOP;

static long prevCount[2]  = {0, 0};
static int  pwmOutput[2]  = {PWM_MIN, PWM_MIN};
static long lastControlMS = 0;

// target distance
static int targetTOF_right = -1;
static int targetTOF_left  = -1;

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
}

/*
Wall following — target-distance P controller

On first tick after entering wall-follow mode, set target_dist = current_dist
  error = current - target.
  error > 0 → robot drifted away from wall - steer back towards wall
  error < 0 → robot drifted into wall      - steer away

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

  int leftPWM  = constrain(WF_BASE_PWM + delta, PWM_MIN, PWM_MAX);
  int rightPWM = constrain(WF_BASE_PWM - delta, PWM_MIN, PWM_MAX);

  // update motor calls
  motor(0, +1, leftPWM);
  motor(1, +1, rightPWM);
}

/*
Left wall (TOF2): error < 0 - more left motor
                  error > 0 - more right motor
*/
void wallFollowLeft() {
  int current = (int)TOF2;
  if (current == 0) current = 255;

  // Latch target distance on first call after entering this mode
  if (targetTOF_left < 0) targetTOF_left = current;

  // error > 0: drifted away from wall - boost right motor
  // error < 0: too close to wall - boost left motor
  int error    = current - targetTOF_left;
  int delta    = constrain((int)(KP_WF * error), -WF_DELTA_MAX, WF_DELTA_MAX);

  int rightPWM = constrain(WF_BASE_PWM + delta, PWM_MIN, PWM_MAX);
  int leftPWM  = constrain(WF_BASE_PWM - delta, PWM_MIN, PWM_MAX);

  // update motor calls
  motor(0, +1, leftPWM);
  motor(1, +1, rightPWM);
}
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
  for (int m = 0; m < 2; m++) {
    noInterrupts();
    long currentCount = encoderCount[m];
    interrupts();

    // calculate current speed from encoder counts
    long measuredSpeed = abs(currentCount - prevCount[m]);
    prevCount[m] = currentCount;

    // adjust pwm output accordingly
    long error      = TARGET_SPEED - measuredSpeed;
    int  correction = (int)(KP * (float)error);
    pwmOutput[m]    = constrain(pwmOutput[m] + correction, PWM_MIN, PWM_MAX);

    motor(m, motorDir, pwmOutput[m]);
  }
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
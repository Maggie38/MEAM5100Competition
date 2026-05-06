// #include "driver.h"
// #include "motor.h"
// #include "tof.h"
// #include "vive.h"
// #include "autonomousattack.h"
// #include <math.h>

// // current drive state
// volatile DriveState driveState = DS_STOP;

// static long prevCount[2]  = {0, 0};
// static long lastControlMS = 0;
// volatile int pwmOutput[2]  = {PWM_MIN, PWM_MIN};

// // target wall follow distance
// static int targetTOF_right = -1;
// static int targetTOF_left  = -1;

// static int wfPrevErrorRight = 0;
// static int wfPrevErrorLeft  = 0;

// // target vive locations
// static float targetX = 0;
// static float targetY = 0;

// // initialize motor pwm to be min
// void driverInit() {
//   for (int m = 0; m < 2; m++) {
//     pwmOutput[m] = PWM_MIN;
//     prevCount[m] = 0;
//   }
//   lastControlMS = millis();
//   targetTOF_right = -1;
//   targetTOF_left  = -1;
// }

// // reset p controller
// void resetPController() {
//   for (int m = 0; m < 2; m++) {
//     pwmOutput[m] = PWM_MIN;
//     noInterrupts();
//     prevCount[m] = encoderCount[m];
//     interrupts();
//   }
// }

// // reset latched wall-follow targets so the next call re-latches
// void resetWallFollow() {
//   targetTOF_right = -1;
//   targetTOF_left  = -1;
//   wfPrevErrorRight = 0;
//   wfPrevErrorLeft  = 0;
// }

// // one step of p controller
// void PController(int motorDir) {
//   // Read both encoder counts atomically
//   noInterrupts();
//   long countL = encoderCount[0];
//   long countR = encoderCount[1];
//   interrupts();

//   // Calculate individual speeds
//   long speedL = abs(countL - prevCount[0]);
//   long speedR = abs(countR - prevCount[1]);
//   prevCount[0] = countL;
//   prevCount[1] = countR;

//   // Keep both motors near TARGET_SPEED using average as the base reading
//   long avgSpeed = (speedL + speedR) / 2;

//   // Base correction: push average toward TARGET_SPEED
//   long baseError      = TARGET_SPEED - avgSpeed;
//   int  baseCorrection = (int)(KP * (float)baseError);

//   // Differential correction: error > 0 means left is faster than right
//   long diffError = speedL - speedR;
//   int  diffCorr  = (int)(KP_DIFF * (float)diffError);

//   // Left motor gets less PWM if it's faster, right gets more
//   // If left is slower, left gets more PWM, right gets less
//   pwmOutput[0] = constrain(pwmOutput[0] + baseCorrection - diffCorr, PWM_MIN, PWM_MAX);
//   pwmOutput[1] = constrain(pwmOutput[1] + baseCorrection + diffCorr, PWM_MIN, PWM_MAX);

//   motor(0, motorDir, pwmOutput[0]);
//   motor(1, motorDir, pwmOutput[1]);
// }

// // update driver if health > 0
// void driverUpdate(byte health) {
//   long now = millis();
//   if (now - lastControlMS < CONTROL_MS) return;
//   lastControlMS = now;

//   if (health == 0) return; 

//   if      (driveState == DS_FORWARD)          PController(+1);
//   else if (driveState == DS_BACKWARD)         PController(-1);
//   else if (driveState == DS_WALL_FOLLOW_RIGHT) wallFollowRight();
//   else if (driveState == DS_WALL_FOLLOW_LEFT)  wallFollowLeft();
//   else if (driveState == DS_AUTO_CIRCUIT)      autoCircuitUpdate();
//   else if (driveState == DS_AUTO_TARGET)       autoTargetUpdate();
//   else if (driveState == DS_AUTO_LOW)   autoLowUpdate();
//   else if (driveState == DS_AUTO_NEXUS) autoNexusUpdate();
//   else if (driveState == DS_AUTO_HIGH)  autoHighUpdate();
// }

// /*
// Wall following — target-distance P controller

// On first tick after entering wall-follow mode, set target_dist = current_dist
//   error = current - target
//   error > 0 - robot drifted away from wall - steer back towards wall
//   error < 0 - robot drifted into wall      - steer away

//   Right wall (TOF1): error < 0 - more right motor
//                      error > 0 - more left motor
// */
// void wallFollowRight() {
//   int current = (int)TOF1;
//   if (current == 0) current = 255; 

//   // Latch target distance on first call after entering this mode
//   if (targetTOF_right < 0) targetTOF_right = current;

//   // error > 0: drifted away from wall - boost left motor
//   // error < 0: too close to wall - boost right motor
//   int error    = current - targetTOF_right;
//   int delta    = constrain((int)(KP_WF * error), -WF_DELTA_MAX, WF_DELTA_MAX);

//   pwmOutput[0] = WF_BASE_PWM;
//   pwmOutput[1] = WF_BASE_PWM;

//     pwmOutput[0] = constrain(WF_BASE_PWM + delta, PWM_MIN, PWM_MAX);
//     pwmOutput[1] = constrain(WF_BASE_PWM - delta, PWM_MIN, PWM_MAX);

//   // update motor calls
//   motor(0, +1, pwmOutput[0]);
//   motor(1, +1, pwmOutput[1]);
// }

// void wallFollowLeft() {
//   int current = (int)TOF2;
//   if (current == 0) current = 255;

//   // Latch target distance on first call after entering this mode
//   if (targetTOF_left < 0) targetTOF_left = current;

//   // error > 0: drifted away from wall - boost right motor
//   // error < 0: too close to wall - boost left motor
//   int error    = current - targetTOF_left;
//   int delta    = constrain((int)(KP_WF * error), -WF_DELTA_MAX, WF_DELTA_MAX);

//   pwmOutput[0] = WF_BASE_PWM;
//   pwmOutput[1] = WF_BASE_PWM;

//     pwmOutput[1] = constrain(WF_BASE_PWM + delta, PWM_MIN, PWM_MAX);
//     pwmOutput[0] = constrain(WF_BASE_PWM - delta, PWM_MIN, PWM_MAX);

//   // update motor calls
//   motor(0, +1, pwmOutput[0]);
//   motor(1, +1, pwmOutput[1]);
// }

// void wallFollowRightSpeed(long targetSpeed) {
//   int current = (int)TOF1;
//   if (current == 0) current = 255;

//   if (targetTOF_right < 0) targetTOF_right = current;

//   // Speed regulation (same as PController but single base)
//   noInterrupts();
//   long countL = encoderCount[0];
//   long countR = encoderCount[1];
//   interrupts();

//   long speedL = abs(countL - prevCount[0]);
//   long speedR = abs(countR - prevCount[1]);
//   prevCount[0] = countL;
//   prevCount[1] = countR;

//   long avgSpeed = (speedL + speedR) / 2;
//   long baseError = targetSpeed - avgSpeed;
//   int baseCorrection = (int)(KP * (float)baseError);

//   // Wall steering correction
//   int error = current - targetTOF_right;
//   int delta = constrain((int)(KP_WF * error), -WF_DELTA_MAX, WF_DELTA_MAX);

//   // Apply speed regulation to both, then wall correction on top
//   pwmOutput[0] = constrain(pwmOutput[0] + baseCorrection, PWM_MIN, PWM_MAX);
//   pwmOutput[1] = constrain(pwmOutput[1] + baseCorrection, PWM_MIN, PWM_MAX);

//   // Steering: delta > 0 drifted away, boost left; delta < 0 too close, boost right
//   if (delta > 0)
//     pwmOutput[0] = constrain(pwmOutput[0] + delta, PWM_MIN, PWM_MAX);
//   else
//     pwmOutput[1] = constrain(pwmOutput[1] - delta, PWM_MIN, PWM_MAX);

//   motor(0, +1, pwmOutput[0]);
//   motor(1, +1, pwmOutput[1]);
// }

// static long acTurnStartPos = 0;

// enum ACStep {
//   AC_WF_1 = 0,   // right wall follow until front < 55
//   AC_TL_1,        // turn left slowly until front > 70
//   AC_STR_1,       // drive straight until front < 50
//   AC_TL_2,        // turn left slowly until front > 70
//   AC_WF_2,        // right wall follow until front < 60
//   AC_TL_3,        // turn left slowly until front > 70
//   AC_STR_2,       // drive straight until front < 50
//   AC_NUM_STEPS
// };

// static ACStep acStep = AC_WF_1;

// void resetAutoCircuit() {
//   acStep = AC_WF_1;
//   acTurnStartPos = 0;
//   resetWallFollow();
//   resetPController();
//   Serial.println("reset auto circuit");
// }

// // Start a left turn: left motor stops, right motor drives forward
// static void startTurnLeftEncoder() {
//   motor(0, -1, 100);
//   motor(1, +1, 100);
//   noInterrupts();
//   acTurnStartPos = encoderCount[1]; // track right motor
//   interrupts();
// }

// static void nextStep() {
//   motorsStop();
//   acTurnStartPos = 0;
//   resetWallFollow();
//   acStep = (ACStep)((int)acStep + 1);
//   if (acStep >= AC_NUM_STEPS) acStep = AC_WF_1;
//   Serial.print("AC next step: "); Serial.println((int)acStep);
// }

// // execute current step of wall follow
// // void autoCircuitUpdate() {
// //   int front = (int)TOF3;
// //   if (front == 0) front = 255;

// //   noInterrupts();
// //   long currentR = encoderCount[1];
// //   interrupts();

// //   switch (acStep) {

// //     case AC_WF_1:
// //     case AC_WF_2:
// //       wallFollowRight();
// //       if (front <= 50) nextStep();
// //       break;

// //     case AC_TL_1:
// //     case AC_TL_2:
// //     case AC_TL_3:
// //     case AC_TL_4:
// //       if (acTurnStartPos == 0) startTurnLeftEncoder();
// //       if (abs(currentR - acTurnStartPos) >= AC_TURN90_TICKS) nextStep();
// //       break;

// //     case AC_STR_1:
// //       // drive straight, no wall follow
// //       motor(0, +1, WF_BASE_PWM);
// //       motor(1, +1, WF_BASE_PWM);
// //       if (front <= 60) nextStep();
// //       break;

// //     case AC_STR_2:
// //       // drive straight, no wall follow
// //       motor(0, +1, WF_BASE_PWM);
// //       motor(1, +1, WF_BASE_PWM);
// //       if (front <= 50) nextStep();
// //       break;
    
// //     case AC_STR_ENTRY: {
// //       if (acTurnStartPos == 0) {
// //         noInterrupts();
// //         acTurnStartPos = encoderCount[1];  // reuse as start reference
// //         interrupts();
// //         resetPController();
// //       }
// //       PController(+1);
// //       if (abs(currentR - acTurnStartPos) >= 100) nextStep();
// //       break;
// //     }

// //     default:
// //       motorsStop();
// //       break;
// //   }
// // }

// void autoCircuitUpdate() {
//   int front = (int)TOF3;
//   if (front == 0) front = 255;

//   switch (acStep) {
//     case AC_WF_1:
//       if (targetTOF_right < 0) resetPController();
//       wallFollowRightSpeed(1500);
//       if (front < 55) nextStep();
//       break;

//     case AC_WF_2:
//       if (targetTOF_right < 0) resetPController();
//       wallFollowRightSpeed(1500);
//       if (front < 60) nextStep();
//       break;

//     case AC_TL_1:
//     case AC_TL_2:
//     case AC_TL_3:
//       // slow turn: right forward, left reverse
//       motor(0, -1, 60);
//       motor(1, +1, 60);
//       if (front > 70) nextStep();
//       break;

//     case AC_STR_1:
//     case AC_STR_2:
//       motor(0, +1, WF_BASE_PWM);
//       motor(1, +1, WF_BASE_PWM);
//       if (front < 50) nextStep();
//       break;

//     default:
//       motorsStop();
//       break;
//   }
// }

// // autonomous vive cover logic
// enum AutoTargetStep {
//   AT_TURN,
//   AT_DRIVE,
//   AT_DONE
// };

// static AutoTargetStep autoTargetStep = AT_DONE;

// constexpr float ANGLE_TOL_DEG = 8.0;     // acceptable heading error
// constexpr float DIST_TOL = 150.0;        // Vive coordinate tolerance
// constexpr int TURN_PWM = 100;
// constexpr int DRIVE_PWM = 115;

// static float angleWrapDeg(float a) {
//   while (a > 180.0f) a -= 360.0f;
//   while (a < -180.0f) a += 360.0f;
//   return a;
// }

// void startAutoTarget(float tx, float ty) {
//   targetX = tx;
//   targetY = ty;
//   autoTargetStep = AT_TURN;
//   resetPController();
//   driveState = DS_AUTO_TARGET;
// }

// void autoTargetUpdate() {
//   float dx = targetX - robotX;
//   float dy = targetY - robotY;

//   float targetAngle = atan2(dy, dx) * 180.0f / PI;
//   float angleError = angleWrapDeg(targetAngle - robotTheta);
//   float dist = sqrt(dx * dx + dy * dy);

//   if (autoTargetStep == AT_TURN) {
//     if (abs(angleError) <= ANGLE_TOL_DEG) {
//       motorsStop();
//       resetPController();
//       autoTargetStep = AT_DRIVE;
//       return;
//     }

//     if (angleError > 0) {
//       // turn left
//       motor(0, -1, TURN_PWM);
//       motor(1, +1, TURN_PWM);
//     } else {
//       // turn right
//       motor(0, +1, TURN_PWM);
//       motor(1, -1, TURN_PWM);
//     }
//   }

//   else if (autoTargetStep == AT_DRIVE) {
//     if (dist <= DIST_TOL) {
//       motorsStop();
//       autoTargetStep = AT_DONE;
//       driveState = DS_STOP;
//       return;
//     }

//     int correction = constrain((int)(1.5f * angleError), -40, 40);

//     int leftPWM  = constrain(DRIVE_PWM - correction, PWM_MIN, PWM_MAX);
//     int rightPWM = constrain(DRIVE_PWM + correction, PWM_MIN, PWM_MAX);

//     motor(0, +1, leftPWM);
//     motor(1, +1, rightPWM);
//   }

//   else {
//     motorsStop();
//     driveState = DS_STOP;
//   }
// }





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
  pwmOutput[0] = constrain(pwmOutput[0] + baseCorrection, PWM_MIN, PWM_MAX);
  pwmOutput[1] = constrain(pwmOutput[1] + baseCorrection, PWM_MIN, PWM_MAX);

  // Steering: delta > 0 drifted away, boost left; delta < 0 too close, boost right
  if (delta > 0)
    pwmOutput[0] = constrain(pwmOutput[0] + delta, PWM_MIN, PWM_MAX);
  else
    pwmOutput[1] = constrain(pwmOutput[1] - delta, PWM_MIN, PWM_MAX);

  motor(0, +1, pwmOutput[0]);
  motor(1, +1, pwmOutput[1]);
}

static long acTurnStartPos = 0;

enum ACStep {
  AC_WF_1 = 0,   // right wall follow until front < 55
  AC_TL_1,        // turn left slowly until front > 70
  AC_STR_1,       // drive straight until front < 50
  AC_TL_2,        // turn left slowly until front > 70
  AC_WF_2,        // right wall follow until front < 60
  AC_TL_3,        // turn left slowly until front > 70
  AC_STR_2,       // drive straight until front < 50
  AC_NUM_STEPS
};

static ACStep acStep = AC_WF_1;

void resetAutoCircuit() {
  acStep = AC_WF_1;
  acTurnStartPos = 0;
  resetWallFollow();
  resetPController();
  Serial.println("reset auto circuit");
}

// Start a left turn: left motor stops, right motor drives forward
static void startTurnLeftEncoder() {
  motor(0, -1, 100);
  motor(1, +1, 100);
  noInterrupts();
  acTurnStartPos = encoderCount[1]; // track right motor
  interrupts();
}

static void nextStep() {
  motorsStop();
  acTurnStartPos = 0;
  resetWallFollow();
  acStep = (ACStep)((int)acStep + 1);
  if (acStep >= AC_NUM_STEPS) acStep = AC_WF_1;
  Serial.print("AC next step: "); Serial.println((int)acStep);
}

// execute current step of wall follow
// void autoCircuitUpdate() {
//   int front = (int)TOF3;
//   if (front == 0) front = 255;

//   noInterrupts();
//   long currentR = encoderCount[1];
//   interrupts();

//   switch (acStep) {

//     case AC_WF_1:
//     case AC_WF_2:
//       wallFollowRight();
//       if (front <= 50) nextStep();
//       break;

//     case AC_TL_1:
//     case AC_TL_2:
//     case AC_TL_3:
//     case AC_TL_4:
//       if (acTurnStartPos == 0) startTurnLeftEncoder();
//       if (abs(currentR - acTurnStartPos) >= AC_TURN90_TICKS) nextStep();
//       break;

//     case AC_STR_1:
//       // drive straight, no wall follow
//       motor(0, +1, WF_BASE_PWM);
//       motor(1, +1, WF_BASE_PWM);
//       if (front <= 60) nextStep();
//       break;

//     case AC_STR_2:
//       // drive straight, no wall follow
//       motor(0, +1, WF_BASE_PWM);
//       motor(1, +1, WF_BASE_PWM);
//       if (front <= 50) nextStep();
//       break;
    
//     case AC_STR_ENTRY: {
//       if (acTurnStartPos == 0) {
//         noInterrupts();
//         acTurnStartPos = encoderCount[1];  // reuse as start reference
//         interrupts();
//         resetPController();
//       }
//       PController(+1);
//       if (abs(currentR - acTurnStartPos) >= 100) nextStep();
//       break;
//     }

//     default:
//       motorsStop();
//       break;
//   }
// }

void autoCircuitUpdate() {
  int front = (int)TOF3;
  if (front == 0) front = 255;

  switch (acStep) {
    case AC_WF_1:
      if (targetTOF_right < 0) resetPController();
      wallFollowRightSpeed(1500);
      if (front < 55) nextStep();
      break;

    case AC_WF_2:
      if (targetTOF_right < 0) resetPController();
      wallFollowRightSpeed(1500);
      if (front < 60) nextStep();
      break;

    case AC_TL_1:
    case AC_TL_2:
    case AC_TL_3:
      // slow turn: right forward, left reverse
      motor(0, -1, 60);
      motor(1, +1, 60);
      if (front > 70) nextStep();
      break;

    case AC_STR_1:
    case AC_STR_2:
      motor(0, +1, WF_BASE_PWM);
      motor(1, +1, WF_BASE_PWM);
      if (front < 50) nextStep();
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

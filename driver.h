#pragma once
#include <Arduino.h>

// P-controller tuning
constexpr long  TARGET_SPEED = 320 * 5;
constexpr float KP           = 0.8f;
constexpr float KP_DIFF      = 0.6f;
constexpr int   CONTROL_MS   = 500; 

constexpr int PWM_MIN = 60;
constexpr int PWM_MAX = 120;

extern volatile int pwmOutput[2];

// Drive states
enum DriveState { 
  DS_STOP, DS_FORWARD, DS_BACKWARD, DS_LEFT, DS_RIGHT,
  DS_WALL_FOLLOW_RIGHT, DS_WALL_FOLLOW_LEFT, DS_AUTO_CIRCUIT,
  DS_AUTO_TARGET
};
extern volatile DriveState driveState;

// full circuit wall follow constants
constexpr int AC_FRONT_WALL_CM = 20;
constexpr int AC_TURN90_TICKS = 200;
constexpr int AC_WEAVE_TICKS = 200;
constexpr int AC_TICKS_30CM = 100;
constexpr int AC_TICKS_20CM = 80;

/**
 * Initialize driver state.
 */
void driverInit();

/**
 * Run one P-controller tick for both motors.
 * motorDir: +1 forward | -1 backward
 * Reads encoderCount[], updates PWM outputs, and calls motor().
 */
void PController(int motorDir);

/**
 * Reset integrator state (previous counts + PWM outputs to min).
 * Call whenever direction changes.
 */
void resetPController();

/**
 * Call from loop().  Checks driveState and fires PController if
 * CONTROL_MS has elapsed and the robot is alive.
 * health: current health byte from TopHat (0 = dead).
 */
void driverUpdate(byte health);

// Wall-follow tuning
constexpr int WF_BASE_PWM  = 100;  // baseline PWM for both motors
constexpr int WF_DELTA_MAX = 40;   // max correction added/subtracted
constexpr float KP_WF      = 0.8f; // proportional gain for wall follow

constexpr uint32_t WF_STRAIGHT_MS  = 300;  // ms driving straight before sampling
constexpr uint32_t WF_CORRECT_MS   = 150;  // ms applying correction
constexpr int      WF_DEAD_BAND    = 2; 

constexpr float KD_WF = 1.0f;

/**
 * Call before entering wall-follow state.
 */
void resetWallFollow();

/**
 * One step of right-wall following (uses TOF1, mounted on right side).
 * Latches target distance on first call; holds correction every tick.
 */
void wallFollowRight();

/**
 * One step of left-wall following (uses TOF2, mounted on left side).
 */
void wallFollowLeft();

/*
 * Starts full circuit wall follow
*/
void startCircuitFollow();

/**
 * Call before entering full circuit wall-follow state.
 */
void resetAutoCircuit();

/**
 * One step of circuit wall follow
 */
void autoCircuitUpdate();

/*
* Start autonomous vive cover logic
*/
void startAutoTarget(float tx, float ty);

/*
* One step of vive cover logic
*/
void autoTargetUpdate();
#pragma once
#include <Arduino.h>

// P-controller tuning
constexpr long  TARGET_SPEED = 100;   // counts / 100 ms
constexpr float KP           = 0.8f;
constexpr int   CONTROL_MS   = 100; 

constexpr int PWM_MIN = 60;
constexpr int PWM_MAX = 255;

// Drive states
enum DriveState { DS_STOP, DS_FORWARD, DS_BACKWARD, DS_LEFT, DS_RIGHT,
                  DS_WALL_FOLLOW_RIGHT, DS_WALL_FOLLOW_LEFT };
extern volatile DriveState driveState;

/**
 * Initialixe driver state.  Call once from setup() after motorInit().
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
constexpr int WF_BASE_PWM  = 120;  // baseline PWM for both motors
constexpr int WF_DELTA_MAX = 80;   // max correction added/subtracted
constexpr float KP_WF      = 1.5f; // proportional gain for wall follow

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
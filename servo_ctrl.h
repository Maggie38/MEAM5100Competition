#pragma once
#include <Arduino.h>

// GPIO and LEDC configuration for the strike servo
constexpr int    SERVO_PIN   = 17;
constexpr int    SERVO_FREQ  = 50;    // Hz
constexpr int    SERVO_RES   = 16;    // bits

// Pulse widths in microseconds
constexpr uint32_t SERVO_REST   = 1000;
constexpr uint32_t SERVO_STRIKE = 2500;

// Time between attack-mode servo state changes
constexpr unsigned long SERVO_ATTACK_INTERVAL_MS = 300;

/** Attach LEDC channel and move servo to rest position. Call once in setup(). */
void servoInit();

/** Write a pulse width (µs) to the servo. */
void servoWrite(uint32_t us);

/**
 * Perform one blocking strike: extend, wait 500 ms, retract.
 */
void servoStrike();

/** Start continuous non-blocking attack mode. */
void servoAttackStart();

/** Stop continuous attack mode and return servo to rest. */
void servoAttackStop();

/** Toggle attack mode. Returns true if attack mode is now active. */
bool servoAttackToggle();

/** Returns true while attack mode is active. */
bool servoAttackIsActive();

/**
 * Non-blocking servo state-machine update.
 * Call this once every loop().
 */
void servoUpdate();
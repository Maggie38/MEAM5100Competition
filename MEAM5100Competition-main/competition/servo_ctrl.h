#pragma once
#include <Arduino.h>

// GPIO and LEDC configuration for the strike servo
constexpr int    SERVO_PIN   = 17;
constexpr int    SERVO_FREQ  = 50;    // Hz
constexpr int    SERVO_RES   = 16;    // bits

// Pulse widths in microseconds
constexpr uint32_t SERVO_REST   = 500;
constexpr uint32_t SERVO_STRIKE = 2000;

/** Attach LEDC channel and move servo to rest position. Call once in setup(). */
void servoInit();

/** Write a pulse width (µs) to the servo. */
void servoWrite(uint32_t us);

/**
 * Perform one strike: extend, wait 500 ms, retract.
 */
void servoStrike();

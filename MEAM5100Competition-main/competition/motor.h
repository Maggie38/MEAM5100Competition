#pragma once
#include <Arduino.h>

// Encoder pins (read-only from this module; written in competition.ino setup)
extern const int ENCA[2];
extern const int ENCB[2];

// Motor driver pins
extern const int IN1[2];
extern const int IN2[2];
extern const int PWM_PIN[2];

// Encoder counts (written by ISRs, read by driver)
extern volatile long encoderCount[2];

/**
 * Configure all motor GPIO pins and attach LEDC channels.
 * Call once from setup().
 */
void motorInit();

/**
 * Drive motor `m` (0 = left, 1 = right).
 * dir: +1 forward | -1 backward | 0 coast-stop
 * spd: 0–255 PWM duty
 */
void motor(int m, int dir, int spd);

/**
 * Stop both motors immediately.
 */
void motorsStop();

// ISR declarations
void IRAM_ATTR encA0_ISR();
void IRAM_ATTR encB0_ISR();
void IRAM_ATTR encA1_ISR();
void IRAM_ATTR encB1_ISR();

#pragma once
#include <Arduino.h>

// XSHUT pins
constexpr int XSHUT_1 = 18;
constexpr int XSHUT_2 = 5;
constexpr int XSHUT_3 = 19;

// Latest distance readings in mm
extern byte TOF1, TOF2, TOF3;

/**
 * Bring up all three VL53L0X sensors on separate I2C addresses.
 */
void tofInit();

/**
 * Read all three sensors and update TOF1/TOF2/TOF3.
 */
void tofUpdate();

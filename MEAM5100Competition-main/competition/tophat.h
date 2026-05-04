#pragma once
#include <Arduino.h>

// I2C address of the TopHat microcontroller
constexpr uint8_t TOPHAT_ADDR = 0x28;

extern byte health;       // 0–100
extern byte packetCount;

/**
 * Read the current health byte from the TopHat over I2C.
 * Update the global health variable.
 */
void tophatReadHealth();

/**
 * Send the accumulated packetCount to the TopHat at 2 Hz, then reset it to 0.
 */
void tophatSendPackets();

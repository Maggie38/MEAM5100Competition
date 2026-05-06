#pragma once
#include <Arduino.h>

// Auto Low Tower
/** Begin autonomous low tower capture sequence. */
void startAutoLow();
/** One tick of the low tower state machine. Called from driverUpdate(). */
void autoLowUpdate();

// Auto Nexus
/** Begin autonomous nexus attack sequence (up to 4 strikes). */
void startAutoNexus();
/** One tick of the nexus state machine. Called from driverUpdate(). */
void autoNexusUpdate();

// Auto High Tower
/** Begin autonomous high tower capture sequence */
void startAutoHigh();
/** One tick of the high tower state machine. Called from driverUpdate(). */
void autoHighUpdate();
#include "tophat.h"
#include <Wire.h>

byte health      = 100;
byte packetCount = 0;

void tophatReadHealth() {
  Wire.requestFrom(TOPHAT_ADDR, (uint8_t)1);
  if (Wire.available()) {
    // update health reading from I2C wire
    health = Wire.read();
  }
}

void tophatSendPackets() {
  Wire.beginTransmission(TOPHAT_ADDR);
  // write packet count for this window
  Wire.write(packetCount);
  Wire.endTransmission();
  // reset packetCount
  packetCount = 0;
}

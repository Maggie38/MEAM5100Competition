/**
 * competition.ino  –  Top-level scheduler
 */

#include <Wire.h>
#include "motor.h"
#include "driver.h"
#include "tophat.h"
#include "tof.h"
#include "servo_ctrl.h"
#include "vive.h"
#include "web_handlers.h"

// I2C bus pins
constexpr int SDA_PIN = 23;
constexpr int SCL_PIN = 22;

constexpr unsigned long PACKET_SEND_MS = 500;
static unsigned long lastPacketSendMS = 0;

void setup() {
  Serial.begin(460800);

  Wire.begin(SDA_PIN, SCL_PIN, 100000);

  motorInit();      // GPIO, LEDC channels, encoder ISRs
  driverInit();     // P-controller state
  servoInit();      // servo LEDC + rest position
  tofInit();        // VL53L0X sensors
  webInit();        // WiFi + HTTP routes
  viveInit();       // setup vive

  motorsStop();

  lastPacketSendMS = millis();
}

void loop() {
  // serve web requests
  webServe();

  // Enforce dead-robot stop every iteration
  if (health == 0) motorsStop();

  // single byte I2C read
  tophatReadHealth();

  // Send packet count to TopHat at 2 Hz
  unsigned long now = millis();
  if (now - lastPacketSendMS >= PACKET_SEND_MS) {
    lastPacketSendMS = now;
    tophatSendPackets();
  }

  // Run one step of P-controller or wall follow depending on current state
  driverUpdate(health);
  // Update TOF readings
  tofUpdate();
  // Update vive localizations
  viveUpdate();

  
}

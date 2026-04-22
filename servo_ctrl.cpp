#include "servo_ctrl.h"

// Convert microseconds to a 16-bit LEDC duty value for a 20 ms period
static uint32_t usToDuty(uint32_t us) {
  return (uint32_t)(us / 20000.0 * 65535);
}

// attach servo pins with frequency and resolution
void servoInit() {
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
  servoWrite(SERVO_REST);
}

// servoWrite
void servoWrite(uint32_t us) {
  ledcWrite(SERVO_PIN, usToDuty(us));
}

// servoStrike
void servoStrike() {
  servoWrite(SERVO_STRIKE);
  delay(500);
  servoWrite(SERVO_REST);
}
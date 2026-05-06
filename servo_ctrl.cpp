#include "servo_ctrl.h"

// Convert microseconds to a 16-bit LEDC duty value for a 20 ms period
static uint32_t usToDuty(uint32_t us) {
  return (uint32_t)(us / 20000.0 * 65535);
}

static bool servoAttackActive = false;
static bool servoAttackExtended = false;
static unsigned long lastServoAttackStepMS = 0;

// attach servo pins with frequency and resolution
void servoInit() {
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
  servoWrite(SERVO_REST);
  servoAttackActive = false;
  servoAttackExtended = false;
  lastServoAttackStepMS = millis();
}

// servoWrite
void servoWrite(uint32_t us) {
  ledcWrite(SERVO_PIN, usToDuty(us));
}

// servoStrike
void servoStrike() {
  // A manual one-shot strike should override continuous attack mode.
  servoAttackStop();
  servoWrite(SERVO_STRIKE);
  delay(500);
  servoWrite(SERVO_REST);
}

void servoAttackStart() {
  servoAttackActive = true;
  servoAttackExtended = false;
  lastServoAttackStepMS = millis();
  servoWrite(SERVO_REST);
}

void servoAttackStop() {
  servoAttackActive = false;
  servoAttackExtended = false;
  servoWrite(SERVO_REST);
}

bool servoAttackToggle() {
  if (servoAttackActive) {
    servoAttackStop();
  } else {
    servoAttackStart();
  }
  return servoAttackActive;
}

bool servoAttackIsActive() {
  return servoAttackActive;
}

void servoUpdate() {
  if (!servoAttackActive) return;

  unsigned long now = millis();
  if (now - lastServoAttackStepMS < SERVO_ATTACK_INTERVAL_MS) return;

  lastServoAttackStepMS = now;
  servoAttackExtended = !servoAttackExtended;

  if (servoAttackExtended) {
    servoWrite(SERVO_STRIKE);
  } else {
    servoWrite(SERVO_REST);
  }
}
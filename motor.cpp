#include "motor.h"

// Encoder pins
const int ENCA[2]    = {35, 39};
const int ENCB[2]    = {34, 36};
// Direction pins
const int IN1[2]     = {25, 32};
const int IN2[2]     = {26, 33};
// PWM pins
const int PWM_PIN[2] = {14, 27};

// Encoder state
volatile long encoderCount[2] = {0, 0};

// ISRs
void IRAM_ATTR encA0_ISR() {
  int a = digitalRead(ENCA[0]), b = digitalRead(ENCB[0]);
  if (a != b) encoderCount[0]++; else encoderCount[0]--;
}
void IRAM_ATTR encB0_ISR() {
  int a = digitalRead(ENCA[0]), b = digitalRead(ENCB[0]);
  if (a == b) encoderCount[0]++; else encoderCount[0]--;
}
void IRAM_ATTR encA1_ISR() {
  int a = digitalRead(ENCA[1]), b = digitalRead(ENCB[1]);
  if (a != b) encoderCount[1]--; else encoderCount[1]++;
}
void IRAM_ATTR encB1_ISR() {
  int a = digitalRead(ENCA[1]), b = digitalRead(ENCB[1]);
  if (a == b) encoderCount[1]--; else encoderCount[1]++;
}

void motorInit() {
  for (int m = 0; m < 2; m++) {
    pinMode(IN1[m], OUTPUT);
    pinMode(IN2[m], OUTPUT);
    pinMode(PWM_PIN[m], OUTPUT);
    ledcAttach(PWM_PIN[m], 1000, 8);   // 1 kHz, 8-bit

    pinMode(ENCA[m], INPUT_PULLUP);
    pinMode(ENCB[m], INPUT_PULLUP);
  }

  // attach all 4 encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA[0]), encA0_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB[0]), encB0_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA[1]), encA1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB[1]), encB1_ISR, CHANGE);
}

// set motor m to have given direction and speed
void motor(int m, int dir, int spd) {
  if (dir > 0) {
    digitalWrite(IN1[m], HIGH); digitalWrite(IN2[m], LOW);
  } else if (dir < 0) {
    digitalWrite(IN1[m], LOW);  digitalWrite(IN2[m], HIGH);
  } else {
    digitalWrite(IN1[m], LOW);  digitalWrite(IN2[m], LOW);
  }
  if (m == 1)
     spd = (int)((float)spd * 1.02);
  ledcWrite(PWM_PIN[m], (dir == 0) ? 0 : spd);
}

void motorsStop() {
  motor(0, 0, 0);
  motor(1, 0, 0);
}

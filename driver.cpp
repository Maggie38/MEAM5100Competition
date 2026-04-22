#include "driver.h"
#include "motor.h"

// current drive state
volatile DriveState driveState = DS_STOP;

static long prevCount[2]  = {0, 0};
static int  pwmOutput[2]  = {PWM_MIN, PWM_MIN};
static long lastControlMS = 0;

// initialize motor pwm to be min
void driverInit() {
  for (int m = 0; m < 2; m++) {
    pwmOutput[m] = PWM_MIN;
    prevCount[m] = 0;
  }
  lastControlMS = millis();
}

// reset p controller
void resetPController() {
  for (int m = 0; m < 2; m++) {
    pwmOutput[m] = PWM_MIN;
    noInterrupts();
    prevCount[m] = encoderCount[m];
    interrupts();
  }
}

// one step of p controller
void PController(int motorDir) {
  for (int m = 0; m < 2; m++) {
    noInterrupts();
    long currentCount = encoderCount[m];
    interrupts();

    // calculate current speed from encoder counts
    long measuredSpeed = abs(currentCount - prevCount[m]);
    prevCount[m] = currentCount;

    // adjust pwm output accordingly
    long error      = TARGET_SPEED - measuredSpeed;
    int  correction = (int)(KP * (float)error);
    pwmOutput[m]    = constrain(pwmOutput[m] + correction, PWM_MIN, PWM_MAX);

    motor(m, motorDir, pwmOutput[m]);
  }
}

// update driver if health > 0
void driverUpdate(byte health) {
  long now = millis();
  if (now - lastControlMS < CONTROL_MS) return;
  lastControlMS = now;

  if (health == 0) return; 

  if      (driveState == DS_FORWARD)  PController(+1);
  else if (driveState == DS_BACKWARD) PController(-1);
}

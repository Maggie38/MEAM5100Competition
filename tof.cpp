#include "tof.h"
#include <Adafruit_VL53L0X.h>

// Sensor objects
static Adafruit_VL53L0X tof1;
static Adafruit_VL53L0X tof2;
static Adafruit_VL53L0X tof3;

// readings
byte TOF1 = 0, TOF2 = 0, TOF3 = 0;

void tofInit() {
  pinMode(XSHUT_1, OUTPUT); digitalWrite(XSHUT_1, LOW);
  pinMode(XSHUT_2, OUTPUT); digitalWrite(XSHUT_2, LOW);
  pinMode(XSHUT_3, OUTPUT); digitalWrite(XSHUT_3, LOW);

  // Bring each sensor up one at a time and assign a unique address
  digitalWrite(XSHUT_1, HIGH); delay(10);
  tof1.begin(); tof1.setAddress(0x30);

  digitalWrite(XSHUT_2, HIGH); delay(10);
  tof2.begin(); tof2.setAddress(0x31);

  digitalWrite(XSHUT_3, HIGH); delay(10);
  tof3.begin(); tof3.setAddress(0x32);
}

// Read one sensor, return distance in mm clamped to byte (0–255).
static byte readSensor(Adafruit_VL53L0X& sensor) {
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    uint16_t mm = measure.RangeMilliMeter;
    return (byte)min((uint16_t)255, mm);
  }
  return 0;
}

void tofUpdate() {
  TOF1 = readSensor(tof1);
  TOF2 = readSensor(tof2);
  TOF3 = readSensor(tof3);
}

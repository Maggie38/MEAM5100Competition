#include "vive.h"
#include "vive510.h"
#include <Arduino.h>
#include <math.h>

#define VIVE_READ_INTERVAL_MS 20
#define VIVE_SYNC_INTERVAL_MS 5000

static Vive510 vive1(VIVE1_PIN);
static Vive510 vive2(VIVE2_PIN);

volatile uint16_t vive1X = 0, vive1Y = 0;
volatile uint16_t vive2X = 0, vive2Y = 0;

volatile float robotX = 0;
volatile float robotY = 0;
volatile float robotTheta = 0; // radians

static uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t mid;
  if      ((a <= b) && (a <= c)) mid = (b <= c) ? b : c;
  else if ((b <= a) && (b <= c)) mid = (a <= c) ? a : c;
  else                           mid = (a <= b) ? a : b;
  return mid;
}

static bool validCoord(uint16_t x, uint16_t y) {
  return !(x < 1000 || x > 8000 || y < 1000 || y > 8000);
}

static void syncOne(Vive510 &vive) {
  if (vive.status() != VIVE_RECEIVING) {
    vive.sync(3);
  }
}

static void doSync() {
  syncOne(vive1);
  syncOne(vive2);
}

static void readVive1() {
  if (vive1.status() != VIVE_RECEIVING) {
    vive1X = 0;
    vive1Y = 0;
    return;
  }

  static uint16_t x0 = 0, x1 = 0, x2 = 0;
  static uint16_t y0 = 0, y1 = 0, y2 = 0;

  x2 = x1; y2 = y1;
  x1 = x0; y1 = y0;

  x0 = vive1.xCoord();
  y0 = vive1.yCoord();

  uint16_t xf = (uint16_t)med3filt(x0, x1, x2);
  uint16_t yf = (uint16_t)med3filt(y0, y1, y2);

  if (validCoord(xf, yf)) {
    vive1X = xf;
    vive1Y = yf;
  } else {
    vive1X = 0;
    vive1Y = 0;
  }
}

static void readVive2() {
  if (vive2.status() != VIVE_RECEIVING) {
    vive2X = 0;
    vive2Y = 0;
    return;
  }

  static uint16_t x0 = 0, x1 = 0, x2 = 0;
  static uint16_t y0 = 0, y1 = 0, y2 = 0;

  x2 = x1; y2 = y1;
  x1 = x0; y1 = y0;

  x0 = vive2.xCoord();
  y0 = vive2.yCoord();

  uint16_t xf = (uint16_t)med3filt(x0, x1, x2);
  uint16_t yf = (uint16_t)med3filt(y0, y1, y2);

  if (validCoord(xf, yf)) {
    vive2X = xf;
    vive2Y = yf;
  } else {
    vive2X = 0;
    vive2Y = 0;
  }
}

static void updateRobotPose() {
  if (vive1X == 0 || vive1Y == 0 || vive2X == 0 || vive2Y == 0) {
    return;
  }

  robotX = 0.5f * ((float)vive1X + (float)vive2X);
  robotY = 0.5f * ((float)vive1Y + (float)vive2Y);

  robotTheta = atan2(
    (float)vive2Y - (float)vive1Y,
    (float)vive2X - (float)vive1X
  );

  robotTheta += PI;

  robotTheta = robotTheta * 180.0f / PI;
}

static void doRead() {
  readVive1();
  readVive2();
  updateRobotPose();
}

void viveInit() {
  vive1.begin();
  vive2.begin();
  doSync();
}

void viveUpdate() {
  uint32_t now = millis();

  static uint32_t lastRead = 0;
  if (now - lastRead >= VIVE_READ_INTERVAL_MS) {
    lastRead = now;
    doRead();
  }

  static uint32_t lastSync = 0;
  if (now - lastSync >= VIVE_SYNC_INTERVAL_MS) {
    lastSync = now;
    doSync();
  }
}
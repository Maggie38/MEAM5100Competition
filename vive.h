#pragma once
#include <Arduino.h>

extern volatile uint16_t vive1X, vive1Y;
extern volatile uint16_t vive2X, vive2Y;

extern volatile float robotX;
extern volatile float robotY;
extern volatile float robotTheta;

void viveInit();
void viveUpdate();
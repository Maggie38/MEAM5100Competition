#include "web_handlers.h"
#include "motor.h"
#include "driver.h"
#include "autonomousattack.h"
#include "tophat.h"
#include "servo_ctrl.h"
#include "html510.h"
#include "motorwebsite.h"
#include "tof.h"
#include "vive.h"

// WiFi credentials
static const char* SSID     = "TP-Link_8A8C";
static const char* PASSWORD = "12488674";

// HTTP server
static HTML510Server h(80);

// Route handlers 
static void handleRoot() {
  h.sendhtml(body);
}

// telemetry handler
static void stateHandler() {
    char json[150];
    sprintf(json,
        "{\"health\":%d,\"x\":%.1f,\"y\":%.1f,\"theta\":%.1f,"
        "\"tof1\":%d,\"tof2\":%d,\"tof3\":%d,"
        "\"pwmL\":%d,\"pwmR\":%d,"
        "\"encL\":%ld,\"encR\":%ld}",
        health, robotX, robotY, robotTheta,
        TOF1, TOF2, TOF3,
        pwmOutput[0], pwmOutput[1],
        encoderCount[0], encoderCount[1]
    );
    h.sendplain(json);
}

static void forwardHandler() {
  if (health > 0) {
    packetCount++;
    if (driveState != DS_FORWARD) resetPController();
    // update drive state
    driveState = DS_FORWARD;
    h.sendplain("forward");
  } else {
    h.sendplain("dead");
  }
}

static void backwardHandler() {
  if (health > 0) {
    packetCount++;
    if (driveState != DS_BACKWARD) resetPController();
    // update drive state
    driveState = DS_BACKWARD;
    h.sendplain("backward");
  } else {
    h.sendplain("dead");
  }
}

static void rightHandler() {
  if (health > 0) {
    packetCount++;
    resetWallFollow();
    driveState = DS_RIGHT;
    // turn right
    motor(1, 0, 0);
    motor(0, 1, 110);
    h.sendplain("right");
  } else {
    h.sendplain("dead");
  }
}

static void leftHandler() {
  if (health > 0) {
    packetCount++;
    resetWallFollow();
    driveState = DS_LEFT;
    // turn left
    motor(1, 1, 110);
    motor(0, 0, 0);
    h.sendplain("left");
  } else {
    h.sendplain("dead");
  }
}

static void strikeHandler() {
  if (health > 0) {
    packetCount++;
    servoStrike();
    h.sendplain("strike");
  } else {
    h.sendplain("dead");
  }
}

static void wallFollowRightHandler() {
  if (health > 0) {
    packetCount++;
    resetWallFollow();
    driveState = DS_WALL_FOLLOW_RIGHT;
    h.sendplain("wall_follow_right");
  } else {
    h.sendplain("dead");
  }
}

static void wallFollowLeftHandler() {
  if (health > 0) {
    packetCount++;
    resetWallFollow();
    driveState = DS_WALL_FOLLOW_LEFT;
    h.sendplain("wall_follow_left");
  } else {
    h.sendplain("dead");
  }
}

static void autoCircuitHandler() {
  if (health > 0) {
    packetCount++;
    resetAutoCircuit();
    driveState = DS_AUTO_CIRCUIT;
    h.sendplain("auto_circuit");
  } else {
    h.sendplain("dead");
  }
}

static void autoLowHandler() {
  if (health > 0) {
    packetCount++;
    startAutoLow();
    h.sendplain("auto_low");
  } else {
    h.sendplain("dead");
  }
}

static void autoNexusHandler() {
  if (health > 0) {
    packetCount++;
    startAutoNexus();
    h.sendplain("auto_nexus");
  } else {
    h.sendplain("dead");
  }
}

static void autoHighHandler() {
  if (health > 0) {
    packetCount++;
    startAutoHigh();
    h.sendplain("auto_high");
  } else {
    h.sendplain("dead");
  }
}

static void attackModeHandler() {
  if (health > 0) {
    packetCount++;
    bool active = servoAttackToggle();
    h.sendplain(active ? "attack_on" : "attack_off");
  } else {
    servoAttackStop();
    h.sendplain("dead");
  }
}

static void stopHandler() {
  packetCount++;
  driveState = DS_STOP;
  motorsStop();
  h.sendplain("stop");
}

static void gotoHandler() {
  if (health <= 0) {
    h.sendplain("dead");
    return;
  }
  packetCount++;

  int tx = h.getVal();   // reads x value
  h.getText();           // consumes "y=" 
  int ty = h.getVal();   // reads y value

  startAutoTarget((float)tx, (float)ty);
  h.sendplain("goto");
}

void webInit() {

  // AP mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Team 11 ESP", "12345678");

  Serial.println("\nAP started!");
  Serial.print("Connect to 'Team 11 ESP' then go to: http://");
  Serial.println(WiFi.softAPIP()); 

  h.begin();

  // attach handlers
  h.attachHandler("/",         handleRoot);
  h.attachHandler("/forward",  forwardHandler);
  h.attachHandler("/backward", backwardHandler);
  h.attachHandler("/left",     leftHandler);
  h.attachHandler("/right",    rightHandler);
  h.attachHandler("/stop",            stopHandler);
  h.attachHandler("/strike",          strikeHandler);
  h.attachHandler("/wallfollowRight", wallFollowRightHandler);
  h.attachHandler("/wallfollowLeft",  wallFollowLeftHandler);
  h.attachHandler("/autocircuit", autoCircuitHandler);
  h.attachHandler("/autolow",     autoLowHandler);
  h.attachHandler("/autonexus",   autoNexusHandler);
  h.attachHandler("/autohigh",    autoHighHandler);
  h.attachHandler("/state", stateHandler);
  h.attachHandler("/goto?x=", gotoHandler);
  h.attachHandler("/attack",      attackModeHandler);
}

void webServe() {
  h.serve();
}
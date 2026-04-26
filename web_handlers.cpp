#include "web_handlers.h"
#include "motor.h"
#include "driver.h"
#include "tophat.h"
#include "servo_ctrl.h"
#include "html510.h"
#include "motorwebsite.h"

// WiFi credentials
static const char* SSID     = "TP-Link_8A8C";
static const char* PASSWORD = "12488674";

// HTTP server
static HTML510Server h(80);

// Route handlers 
static void handleRoot() {
  h.sendhtml(body);
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
    motor(0, 0, 0);
    motor(1, 1, 110);
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
    motor(0, 1, 110);
    motor(1, 0, 0);
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
    driveState = DS_WALL_FOLLOW_RIGHT;
    h.sendplain("wall_follow_right");
  } else {
    h.sendplain("dead");
  }
}

static void wallFollowLeftHandler() {
  if (health > 0) {
    packetCount++;
    driveState = DS_WALL_FOLLOW_LEFT;
    h.sendplain("wall_follow_left");
  } else {
    h.sendplain("dead");
  }
}

static void stopHandler() {
  packetCount++;
  driveState = DS_STOP;
  motorsStop();
  h.sendplain("stop");
}

void webInit() {
  // connect to wifi
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("Go to: http://");
  Serial.println(WiFi.localIP());

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
}

void webServe() {
  h.serve();
}
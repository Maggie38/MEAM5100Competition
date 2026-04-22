#pragma once
#include <Arduino.h>

/**
 * Connect to WiFi, start the HTTP server, and register all route handlers.
 */
void webInit();

/**
 * Process one pending HTTP request.
 */
void webServe();

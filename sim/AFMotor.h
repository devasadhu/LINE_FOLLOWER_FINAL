// Host-side stand-in for the Adafruit Motor Shield library: records speed and direction per port.
#pragma once
#include "Arduino.h"
#define FORWARD 1
#define BACKWARD 2
#define RELEASE 4
#define MOTOR12_64KHZ 1
#define MOTOR34_1KHZ 3
#define MOTOR34_64KHZ 1
struct AF_DCMotor {
  static AF_DCMotor *reg[5];
  int port, freq, speed = 0, state = RELEASE;
  AF_DCMotor(uint8_t n, uint8_t f) : port(n), freq(f) { reg[n] = this; }
  void setSpeed(uint8_t s) { speed = s; }
  void run(uint8_t c) { state = c; }
};

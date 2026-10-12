// Minimal host-side stand-ins for the Arduino core, just enough to compile the sketches on a PC.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define INPUT 0
struct __FlashStringHelper;
#define F(x) (reinterpret_cast<const __FlashStringHelper *>(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
inline void pinMode(uint8_t, uint8_t) {}
int analogRead(uint8_t pin);
unsigned long millis();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int);
struct SerialStub {
  bool verbose = false;
  void begin(long) {}
  void print(const __FlashStringHelper *s) { if (verbose) printf("%s", (const char *)s); }
  void println(const __FlashStringHelper *s) { if (verbose) printf("%s\n", (const char *)s); }
  void print(const char *s) { if (verbose) printf("%s", s); }
  void println(const char *s) { if (verbose) printf("%s\n", s); }
  void print(int v) { if (verbose) printf("%d", v); }
  void println(int v) { if (verbose) printf("%d\n", v); }
  void print(unsigned long v) { if (verbose) printf("%lu", v); }
  void println(unsigned long v) { if (verbose) printf("%lu\n", v); }
  void print(float v, int d = 2) { if (verbose) printf("%.*f", d, v); }
  void println(float v, int d = 2) { if (verbose) printf("%.*f\n", d, v); }
};
extern SerialStub Serial;

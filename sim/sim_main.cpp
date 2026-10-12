// Software-in-the-loop check: runs the REAL line_follower.ino against a simple simulated robot.
// Physics is an approximation (assumed wheel speed, wheelbase and sensor geometry), so this
// catches sign errors, instability and logic bugs. It does not replace tuning on the real robot.
//
// Build:  g++ -std=c++17 -O1 -I. -I../line_follower -o sim sim_main.cpp
// Run:    ./sim
#include "Arduino.h"
#include "AFMotor.h"
#include "line_follower.ino"
#include <string>

SerialStub Serial;
AF_DCMotor *AF_DCMotor::reg[5] = {0, 0, 0, 0, 0};

static unsigned long simMs = 0;
unsigned long millis() { return simMs; }
void delay(unsigned long ms) { simMs += ms; }
void delayMicroseconds(unsigned int) {}

// ---- Simulated world and robot (all lengths in metres) -----------------------------------
struct Scenario {
  const char *name;
  double amp, wavelength;        // line is y = amp * sin(2*pi*x / wavelength)
  double halfWindow;             // half-width of the region a sensor reads as "black"
  double x0, y0, heading0;       // initial pose
  double lineStart, lineEnd;     // line exists only for lineStart <= x <= lineEnd
  double boxStart, boxEnd;       // all-black region (finish box)
  double maxTime;                // seconds
};

static const double WHEELBASE = 0.13;     // assumed
static const double MAX_WHEEL_V = 0.6;    // m/s at PWM 255 (assumed)
static const double SENSOR_AHEAD = 0.07;  // sensor bar distance ahead of the axle
static const double SENSOR_PITCH = 0.03;  // spacing between sensors

static Scenario S;
static double rx, ry, rth;

static double lineY(double x)  { return S.amp * sin(2 * M_PI * x / S.wavelength); }
static double lineDY(double x) { return S.amp * (2 * M_PI / S.wavelength) * cos(2 * M_PI * x / S.wavelength); }

static double sensorDistance(int idx) {   // idx 0 = leftmost sensor
  double lateral = (2 - idx) * SENSOR_PITCH;
  double px = rx + SENSOR_AHEAD * cos(rth) - lateral * sin(rth);
  double py = ry + SENSOR_AHEAD * sin(rth) + lateral * cos(rth);
  if (px >= S.boxStart && px <= S.boxEnd) return 0.0;
  if (px < S.lineStart || px > S.lineEnd) return 1e9;
  return fabs(py - lineY(px)) / sqrt(1 + lineDY(px) * lineDY(px));
}

int analogRead(uint8_t pin) {
  for (int i = 0; i < SENSOR_COUNT; i++)
    if (SENSOR_PINS[i] == pin) return sensorDistance(i) < S.halfWindow ? 900 : 100;
  return 0;
}

static double wheelVelocity(const AF_DCMotor *m) {
  if (m->state == FORWARD)  return  m->speed / 255.0 * MAX_WHEEL_V;
  if (m->state == BACKWARD) return -m->speed / 255.0 * MAX_WHEEL_V;
  return 0.0;
}

static void run(const Scenario &sc) {
  S = sc;
  rx = sc.x0; ry = sc.y0; rth = sc.heading0;
  lastError = 0; integral = 0; lastLeft = lastRight = 0;
  lost = allBlack = stopped = false;
  lostSince = allBlackSince = lastControlMs = lastDebugMs = 0;
  simMs = 0;
  setup();

  double maxDev = 0, stopTime = 0;
  unsigned long t0 = simMs;
  while ((simMs - t0) < sc.maxTime * 1000) {
    double vl = wheelVelocity(AF_DCMotor::reg[LEFT_PORT]);
    double vr = wheelVelocity(AF_DCMotor::reg[RIGHT_PORT]);
    double v = (vl + vr) / 2, w = (vr - vl) / WHEELBASE, dt = 0.001;
    rx += v * cos(rth) * dt;  ry += v * sin(rth) * dt;  rth += w * dt;
    simMs++;
    loop();
    if (rx >= sc.lineStart && rx <= sc.lineEnd) {
      double dev = fabs(ry - lineY(rx)) / sqrt(1 + lineDY(rx) * lineDY(rx));
      if (dev > maxDev) maxDev = dev;
    }
    if (stopped && stopTime == 0) stopTime = (simMs - t0) / 1000.0;
    if (rx > 3.0) break;
  }
  printf("%-50s max dev %5.1f mm | end x %5.2f m | %s", sc.name, maxDev * 1000, rx,
         stopped ? "stopped" : "running");
  if (stopped) printf(" at %.1f s", stopTime);
  printf("\n");
}

int main() {
  const double INF = 1e9, NEG = -1e9;
  Scenario list[] = {
    // name, amp, wavelength, halfWindow, x0, y0, heading0, lineStart, lineEnd, boxStart, boxEnd, maxTime
    {"straight, 30 mm offset, 0.2 rad heading",        0.00, 1.0, 0.0175, 0, 0.03, 0.2, NEG, INF, INF, INF, 15},
    {"straight, narrow line (gaps between sensors)",   0.00, 1.0, 0.0145, 0, 0.03, 0.2, NEG, INF, INF, INF, 15},
    {"gentle curve, 100 mm amplitude, 1.2 m period",   0.10, 1.2, 0.0175, 0, 0.00, 0.0, NEG, INF, INF, INF, 20},
    {"tight curve, 100 mm amplitude, 0.6 m period",    0.10, 0.6, 0.0175, 0, 0.00, 0.0, NEG, INF, INF, INF, 25},
    {"finish box at x = 1.0..1.3 m",                   0.00, 1.0, 0.0175, 0, 0.00, 0.0, NEG, INF, 1.0, 1.3, 10},
    {"no line anywhere (lost-line timeout)",           0.00, 1.0, 0.0175, 0, 0.00, 0.0, INF, INF, INF, INF, 6},
    {"dead end at x = 0.8 m (spins, re-finds line)",   0.00, 1.0, 0.0175, 0, 0.00, 0.0, 0.0, 0.8, INF, INF, 8},
  };
  for (const Scenario &sc : list) run(sc);
  return 0;
}

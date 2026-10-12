/*
 * line_follower.ino  --  5-sensor PID line follower
 * Hardware: Arduino Uno + Adafruit Motor Shield v1 (AFMotor library) + 5 IR sensors
 *
 * Improvements over the earlier if/else versions:
 *   - Weighted-position error -> PID, so steering is proportional, not on/off
 *   - Speed drops automatically on sharp corrections
 *   - Line-loss handling: bridge small gaps, then spin toward the side the line
 *     was last seen, then stop safely after a timeout
 *   - All-black handling: crossings are driven straight, a finish box stops the robot
 *   - Fixed-rate control loop with millis() (no blocking delay() inside loop)
 *   - Correct PWM-frequency constants, so millis()/delay() stay accurate on M3/M4
 *   - Rate-limited serial telemetry for tuning
 *
 * Tune in this order: sensors (sensor_test) -> motors (motor_test) -> BASE_SPEED -> Kp -> Kd -> Ki.
 */
#include <AFMotor.h>

// ============================ USER CONFIG ====================================

// ---- Motors ----------------------------------------------------------------
#define LEFT_PORT   3            // motor shield port for the left motor  (1..4)
#define RIGHT_PORT  4            // motor shield port for the right motor (1..4)
const bool LEFT_INVERT  = false; // set true if that wheel spins the wrong way
const bool RIGHT_INVERT = false;

// PWM frequency. IMPORTANT: ports 3/4 share Timer0 with millis()/delay().
// MOTOR34_64KHZ (no prescale) makes millis()/delay() run about 64x too fast.
// MOTOR34_1KHZ keeps Timer0's normal prescaler, so timing stays correct.
// Ports 1/2 use Timer2, so the quiet 64 kHz setting is safe there.
#if LEFT_PORT <= 2
  #define LEFT_PWM_FREQ  MOTOR12_64KHZ
#else
  #define LEFT_PWM_FREQ  MOTOR34_1KHZ
#endif
#if RIGHT_PORT <= 2
  #define RIGHT_PWM_FREQ MOTOR12_64KHZ
#else
  #define RIGHT_PWM_FREQ MOTOR34_1KHZ
#endif

// ---- Sensors (listed LEFT -> RIGHT as seen from behind the robot) -----------
#define SENSOR_COUNT 5
const uint8_t SENSOR_PINS[SENSOR_COUNT]   = {A5, A4, A2, A1, A0};
const float   SENSOR_WEIGHT[SENSOR_COUNT] = {-2, -1, 0, 1, 2};
const bool    LINE_IS_HIGH = true;   // true: black line reads HIGH (typical LM393 IR modules)
const int     THRESHOLD    = 500;    // analogRead() 0..1023; digital modules read ~0 or ~1023

// ---- Speed -------------------------------------------------------------------
const int   BASE_SPEED    = 140;     // cruise PWM (0..255). Start low, raise once stable
const int   MAX_SPEED     = 220;     // forward limit
const int   MAX_REVERSE   = 120;     // reverse limit (lets sharp turns pivot)
const int   MIN_DRIVE     = 50;      // below this PWM the motors only hum, so coast instead
const float TURN_SLOWDOWN = 25.0;    // PWM removed from BASE_SPEED per unit of |error|
const int   SEARCH_SPEED  = 120;     // pivot speed when the line is lost

// ---- PID (error range is -2.0 .. +2.0; output is PWM) -------------------------
float Kp = 45.0;
float Ki = 0.0;                      // keep 0 until Kp/Kd are good
float Kd = 110.0;                    // derivative per control cycle
const float INTEGRAL_LIMIT = 40.0;

// ---- Timing --------------------------------------------------------------------
const unsigned long CONTROL_PERIOD_MS = 10;    // control loop period
const unsigned long START_DELAY_MS    = 3000;  // time to place the robot after power-on
const unsigned long GAP_HOLD_MS       = 80;    // keep last speeds across short line gaps
const unsigned long LOST_TIMEOUT_MS   = 1500;  // stop if the line is not re-found
const bool          STOP_ON_ALL_BLACK = true;  // stop at a finish box (all 5 sensors black)
const unsigned long FINISH_MS         = 300;   // all-black must last this long to count

// ---- Debug ---------------------------------------------------------------------
const bool          DEBUG           = true;
const unsigned long DEBUG_PERIOD_MS = 100;

// ============================ END USER CONFIG ================================

AF_DCMotor leftMotor(LEFT_PORT,   LEFT_PWM_FREQ);
AF_DCMotor rightMotor(RIGHT_PORT, RIGHT_PWM_FREQ);

// Controller state
float lastError  = 0.0;
float integral   = 0.0;
int   lastLeft   = 0;
int   lastRight  = 0;
bool  lost       = false;
bool  allBlack   = false;
bool  stopped    = false;
unsigned long lostSince     = 0;
unsigned long allBlackSince = 0;
unsigned long lastControlMs = 0;
unsigned long lastDebugMs   = 0;

// Drive one motor with a signed speed (-255..255).
void drive(AF_DCMotor &m, int speed, bool invert) {
  if (abs(speed) < MIN_DRIVE) {
    m.run(RELEASE);
    return;
  }
  bool forward = (speed > 0) != invert;
  m.setSpeed(abs(speed));
  m.run(forward ? FORWARD : BACKWARD);
}

void stopMotors() {
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
  lastLeft = 0;
  lastRight = 0;
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) pinMode(SENSOR_PINS[i], INPUT);
  stopMotors();
  Serial.println(F("Line follower ready. Starting soon..."));
  delay(START_DELAY_MS);
  lastControlMs = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastControlMs < CONTROL_PERIOD_MS) return;   // fixed-rate loop
  lastControlMs = now;

  if (stopped) {
    stopMotors();
    return;
  }

  // ---- 1. Read sensors ---------------------------------------------------------
  int   raw[SENSOR_COUNT];
  uint8_t count = 0;
  float sumW = 0.0;
  char  bits[SENSOR_COUNT + 1];
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    raw[i] = analogRead(SENSOR_PINS[i]);
    bool on = LINE_IS_HIGH ? (raw[i] > THRESHOLD) : (raw[i] < THRESHOLD);
    bits[i] = on ? '1' : '0';
    if (on) {
      count++;
      sumW += SENSOR_WEIGHT[i];
    }
  }
  bits[SENSOR_COUNT] = '\0';

  // ---- 2. Decide what to do ------------------------------------------------------
  int   leftSpeed  = 0;
  int   rightSpeed = 0;
  float error      = lastError;
  float correction = 0.0;

  if (count == 0) {
    // Line lost: bridge a short gap, then search toward the last-seen side, then give up.
    if (!lost) {
      lost = true;
      lostSince = now;
      integral = 0.0;
    }
    allBlack = false;
    unsigned long lostFor = now - lostSince;
    if (lostFor < GAP_HOLD_MS) {
      leftSpeed  = lastLeft;
      rightSpeed = lastRight;
    } else if (lostFor < LOST_TIMEOUT_MS) {
      int dir = (lastError >= 0) ? 1 : -1;     // +1: line last seen on the right
      leftSpeed  =  dir * SEARCH_SPEED;
      rightSpeed = -dir * SEARCH_SPEED;
    } else {
      stopped = true;
      stopMotors();
      Serial.println(F("Line lost. Stopped."));
      return;
    }
  } else {
    if (lost) {                                // just re-acquired: avoid a derivative kick
      lost = false;
      lastError = sumW / count;
      integral = 0.0;
    }

    if (count == SENSOR_COUNT) {
      // All sensors black: crossing or finish box. Drive straight; stop if it persists.
      if (!allBlack) {
        allBlack = true;
        allBlackSince = now;
      }
      if (STOP_ON_ALL_BLACK && (now - allBlackSince >= FINISH_MS)) {
        stopped = true;
        stopMotors();
        Serial.println(F("Finish detected. Stopped."));
        return;
      }
      leftSpeed  = BASE_SPEED;
      rightSpeed = BASE_SPEED;
    } else {
      allBlack = false;

      // ---- 3. PID on weighted position ------------------------------------------
      error = sumW / count;                    // -2.0 (far left) .. +2.0 (far right)
      float derivative = error - lastError;
      integral = constrain(integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
      correction = Kp * error + Ki * integral + Kd * derivative;
      lastError = error;

      float base = BASE_SPEED - TURN_SLOWDOWN * fabs(error);
      // Line to the right (error > 0): left wheel faster, right wheel slower.
      leftSpeed  = (int)(base + correction);
      rightSpeed = (int)(base - correction);
    }
  }

  // ---- 4. Drive ---------------------------------------------------------------------
  leftSpeed  = constrain(leftSpeed,  -MAX_REVERSE, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_REVERSE, MAX_SPEED);
  drive(leftMotor,  leftSpeed,  LEFT_INVERT);
  drive(rightMotor, rightSpeed, RIGHT_INVERT);
  lastLeft  = leftSpeed;
  lastRight = rightSpeed;

  // ---- 5. Telemetry --------------------------------------------------------------------
  if (DEBUG && (now - lastDebugMs >= DEBUG_PERIOD_MS)) {
    lastDebugMs = now;
    Serial.print(bits);
    Serial.print(F("  err="));  Serial.print(error, 2);
    Serial.print(F("  corr=")); Serial.print(correction, 1);
    Serial.print(F("  L="));    Serial.print(leftSpeed);
    Serial.print(F("  R="));    Serial.println(rightSpeed);
  }
}

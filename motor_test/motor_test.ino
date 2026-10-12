/*
 * motor_test.ino  --  check motor wiring and timer settings BEFORE running the follower
 * Lift the robot so the wheels are off the ground. Open Serial Monitor at 115200 baud.
 *
 * Part 1: timer check. Ports 3/4 share Timer0 with millis()/delay(). A wrong PWM
 *         frequency constant makes timing run far too fast. This measures it.
 * Part 2: runs each wheel forward and backward, then both forward.
 *         If a wheel turns the wrong way, set LEFT_INVERT / RIGHT_INVERT in line_follower.ino.
 */
#include <AFMotor.h>

#define LEFT_PORT   3
#define RIGHT_PORT  4
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

AF_DCMotor leftMotor(LEFT_PORT,   LEFT_PWM_FREQ);
AF_DCMotor rightMotor(RIGHT_PORT, RIGHT_PWM_FREQ);

const int TEST_SPEED = 150;

void runStep(const __FlashStringHelper *name, int l, int r) {
  Serial.println(name);
  leftMotor.setSpeed(abs(l));
  rightMotor.setSpeed(abs(r));
  leftMotor.run(l == 0 ? RELEASE : (l > 0 ? FORWARD : BACKWARD));
  rightMotor.run(r == 0 ? RELEASE : (r > 0 ? FORWARD : BACKWARD));
  delay(1000);
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
  delay(500);
}

void setup() {
  Serial.begin(115200);

  // Part 1: 200 x 1000 us of cycle-counted delay = 200 ms of real time.
  unsigned long t0 = millis();
  for (int i = 0; i < 200; i++) delayMicroseconds(1000);
  unsigned long dt = millis() - t0;
  Serial.print(F("Timer check: millis() advanced "));
  Serial.print(dt);
  Serial.println(F(" ms for ~200 ms of real time"));
  if (dt >= 180 && dt <= 230) {
    Serial.println(F("  OK: millis()/delay() are accurate."));
  } else {
    Serial.println(F("  PROBLEM: timing is off. Check the PWM frequency constants (use MOTOR34_1KHZ on ports 3/4)."));
  }
  delay(1000);
}

void loop() {
  runStep(F("LEFT  forward"),  TEST_SPEED, 0);
  runStep(F("LEFT  backward"), -TEST_SPEED, 0);
  runStep(F("RIGHT forward"),  0, TEST_SPEED);
  runStep(F("RIGHT backward"), 0, -TEST_SPEED);
  runStep(F("BOTH  forward (robot should drive straight ahead)"), TEST_SPEED, TEST_SPEED);
  Serial.println(F("--- repeat in 3 s ---"));
  delay(3000);
}

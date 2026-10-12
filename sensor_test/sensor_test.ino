/*
 * sensor_test.ino  --  check the 5 IR sensors BEFORE running the follower
 * No motors are used. Open Serial Monitor at 115200 baud.
 *
 * What to check:
 *   1. Order: slide a strip of black tape under each sensor in turn.
 *      The matching column must change, and the order must be LEFT -> RIGHT.
 *      If not, edit SENSOR_PINS in line_follower.ino.
 *   2. Polarity: black should read HIGH (~1023 on digital modules) and white LOW (~0).
 *      If it is the other way round, set LINE_IS_HIGH = false in line_follower.ino.
 *   3. Threshold: for analog-style readings, pick a value between the white and black
 *      readings. Digital modules switch at the potentiometer setting, so turn the small
 *      blue pot until the module LED changes exactly at the black/white boundary.
 */
const uint8_t SENSOR_PINS[5] = {A5, A4, A2, A1, A0};   // left -> right (same as the main sketch)
const bool    LINE_IS_HIGH   = true;
const int     THRESHOLD      = 500;

void setup() {
  Serial.begin(115200);
  Serial.println(F("S1(left)  S2  S3(centre)  S4  S5(right)   |  line?"));
}

void loop() {
  char bits[6];
  for (uint8_t i = 0; i < 5; i++) {
    int raw = analogRead(SENSOR_PINS[i]);
    bool on = LINE_IS_HIGH ? (raw > THRESHOLD) : (raw < THRESHOLD);
    bits[i] = on ? '1' : '0';
    Serial.print(raw);
    Serial.print(F("\t"));
  }
  bits[5] = '\0';
  Serial.print(F("|  "));
  Serial.println(bits);
  delay(150);
}

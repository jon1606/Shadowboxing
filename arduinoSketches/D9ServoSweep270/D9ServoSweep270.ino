// Sweeps the 270-degree radar servo on D3 back and forth, 50-245 deg (the D9 sweep).
#include <Servo.h>

// Pulse widths for 0 and 270 deg. Most 270-degree servos use 500-2500 us; adjust if the ends
// fall short or the servo buzzes against its stop.
const int PULSE_MIN = 500;
const int PULSE_MAX = 2500;
const int SERVO_RANGE = 270;  // full travel of the servo
const int SWEEP_MIN = 50;     // sweep ends, as in D9.cpp
const int SWEEP_MAX = 245;

Servo radar;

// Moves the servo to an angle from 0 to 270.
void moveTo(int angle) {
  radar.writeMicroseconds(map(angle, 0, SERVO_RANGE, PULSE_MIN, PULSE_MAX));
}

void setup() {
  radar.attach(3, PULSE_MIN, PULSE_MAX);
}

void loop() {
  for (int angle = SWEEP_MIN; angle <= SWEEP_MAX; angle++) {
    moveTo(angle);
    delay(10);
  }
  for (int angle = SWEEP_MAX; angle >= SWEEP_MIN; angle--) {
    moveTo(angle);
    delay(10);
  }
}

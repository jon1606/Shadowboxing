// Sweeps the 270-degree radar servo on D3 back and forth, 0-180 deg.
#include <Servo.h>

// Pulse widths for 0 and 270 deg. Most 270-degree servos use 500-2500 us; adjust if the ends
// fall short or the servo buzzes against its stop.
const int PULSE_MIN = 500;
const int PULSE_MAX = 2500;
const int SERVO_RANGE = 270;  // full travel of the servo
const int SWEEP_MAX = 180;    // how far the sweep goes

Servo radar;

// Moves the servo to an angle from 0 to 270.
void moveTo(int angle) {
  radar.writeMicroseconds(map(angle, 0, SERVO_RANGE, PULSE_MIN, PULSE_MAX));
}

void setup() {
  radar.attach(3, PULSE_MIN, PULSE_MAX);
}

void loop() {
  for (int angle = 0; angle <= SWEEP_MAX; angle++) {
    moveTo(angle);
    delay(10);
  }
  for (int angle = SWEEP_MAX; angle >= 0; angle--) {
    moveTo(angle);
    delay(10);
  }
}

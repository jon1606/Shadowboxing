// Sweeps a 180-degree servo on D3 back and forth, 0-180 deg. Nothing else.
// The D9 radar is a 270-degree servo: use D9ServoSweep270 for it.
#include <Servo.h>

Servo radar;

void setup() {
  radar.attach(3);
}

void loop() {
  for (int angle = 0; angle <= 180; angle++) {
    radar.write(angle);
    delay(17);
  }
  for (int angle = 180; angle >= 0; angle--) {
    radar.write(angle);
    delay(17);
  }
}

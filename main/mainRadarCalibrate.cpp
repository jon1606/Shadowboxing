// Radar sweep calibration (270-degree servo on D3): find the two ends the radar should sweep between.
// Copy this file over src/main.cpp, upload, open the Serial Monitor at 9600 and keep USB plugged in.
// The servo needs the battery (6 V servo rail). The tracks are held off.
//
// Angles use the same scale as the D9 code (0-270 over 500-2500 us). It starts at 90 deg, the
// middle of today's 0-180 sweep. Type a number and press Enter:
//   1 / 2   move 5 deg down / up
//   3 / 4   move 1 deg down / up
//   5       save this angle as the LOW end of the sweep
//   6       save this angle as the HIGH end of the sweep
//   7       test: sweep LOW -> HIGH -> LOW twice
//   0       print the angle and the saved ends
// Send the printed LOW/HIGH values; they replace RADAR_MIN_ANGLE / RADAR_MAX_ANGLE in D9.cpp.
#include <Arduino.h>
#include <Servo.h>

const uint8_t RADAR_PIN = 3;
const int SERVO_RANGE = 270;
const int PULSE_MIN = 500;
const int PULSE_MAX = 2500;
const unsigned long STEP_MS = 17;   // same sweep speed as the D9: 1 deg every 17 ms

// Track speed pins, only used to keep the motors off.
const uint8_t LEFT_EN = 6;
const uint8_t RIGHT_EN = 5;

Servo radar;
int angle = 90;
int lowEnd = 0;     // today's sweep, until you save new ends
int highEnd = 180;

void writeAngle(int degrees) {
  radar.writeMicroseconds(map(degrees, 0, SERVO_RANGE, PULSE_MIN, PULSE_MAX));
}

void printState() {
  Serial.print(F("angle="));
  Serial.print(angle);
  Serial.print(F("  LOW="));
  Serial.print(lowEnd);
  Serial.print(F("  HIGH="));
  Serial.print(highEnd);
  Serial.print(F("  (sweep width "));
  Serial.print(highEnd - lowEnd);
  Serial.println(F(" deg)"));
}

// Moves one degree at a time, at the D9's sweep speed.
void moveTo(int target) {
  target = constrain(target, 0, SERVO_RANGE);
  while (angle != target) {
    angle += target > angle ? 1 : -1;
    writeAngle(angle);
    delay(STEP_MS);
  }
}

void setup() {
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
  Serial.begin(9600);
  writeAngle(angle);
  radar.attach(RADAR_PIN, PULSE_MIN, PULSE_MAX);
  Serial.println(F("Radar calibration (D3)"));
  Serial.println(F("1/2 5 deg down/up | 3/4 1 deg down/up | 5 save LOW | 6 save HIGH | 7 test sweep | 0 print"));
  printState();
}

void loop() {
  if (!Serial.available()) {
    return;
  }
  char c = Serial.read();
  if (c == '1') {
    moveTo(angle - 5);
  } else if (c == '2') {
    moveTo(angle + 5);
  } else if (c == '3') {
    moveTo(angle - 1);
  } else if (c == '4') {
    moveTo(angle + 1);
  } else if (c == '5') {
    lowEnd = angle;
  } else if (c == '6') {
    highEnd = angle;
  } else if (c == '7') {
    Serial.println(F("Test sweep LOW -> HIGH -> LOW, twice"));
    for (uint8_t i = 0; i < 2; i++) {
      moveTo(lowEnd);
      moveTo(highEnd);
    }
    moveTo(lowEnd);
  } else if (c != '0') {
    return;
  }
  printState();
}

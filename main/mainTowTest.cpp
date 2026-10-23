// Tow test for the FS90R continuous servo on D11: checks the down and up timings.
// Copy this file over src/main.cpp, upload, open the Serial Monitor at 9600 and keep USB plugged in.
// The servo needs the battery (6 V servo rail); unplug the battery at any time to stop it.
// The tracks are held off. The servo goes limp after every move.
//
// Start with the hook fully UP. Type a letter or number and press Enter:
//   d   DOWN: lower the hook (downMs at downSpeed)
//   u   UP:   lift the hook  (upMs at upSpeed)
//   c   cycle: down, wait 1 s, up
//   1/2 down time  -10 / +10 ms
//   3/4 up time    -100 / +100 ms
//   5/6 up speed   -10 / +10 (up struggles: more speed = more force)
//   7/8 down speed -10 / +10
//   p   print the settings
#include <Arduino.h>
#include <Servo.h>

const uint8_t TOW_PIN = 11;
const int STOP_US = 1500;   // the servo stands still at this pulse

// Track speed pins, only used to keep the motors off.
const uint8_t LEFT_EN = 6;
const uint8_t RIGHT_EN = 5;

// Measured with the tow calibration: 150 ms down from the top hooks it; up takes about 1.5 s.
long downMs = 150;
long upMs = 1500;
int downSpeed = 60;         // pulse distance from STOP_US; down = STOP_US - speed
int upSpeed = 60;           // up = STOP_US + speed

Servo tow;

void printSettings() {
  Serial.print(F("down: "));
  Serial.print(downMs);
  Serial.print(F(" ms at speed "));
  Serial.print(downSpeed);
  Serial.print(F("  |  up: "));
  Serial.print(upMs);
  Serial.print(F(" ms at speed "));
  Serial.println(upSpeed);
}

// Sends one pulse width for ms milliseconds, then stops and goes limp.
void pulseFor(int microseconds, long ms) {
  tow.writeMicroseconds(microseconds);
  tow.attach(TOW_PIN);
  delay(ms);
  tow.writeMicroseconds(STOP_US);
  delay(40);
  tow.detach();
}

void moveDown() {
  Serial.print(F("DOWN "));
  Serial.print(downMs);
  Serial.println(F(" ms"));
  pulseFor(STOP_US - downSpeed, downMs);
}

void moveUp() {
  Serial.print(F("UP "));
  Serial.print(upMs);
  Serial.println(F(" ms"));
  pulseFor(STOP_US + upSpeed, upMs);
}

void setup() {
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
  Serial.begin(9600);
  Serial.println(F("FS90R tow test (D11). Start with the hook fully UP."));
  Serial.println(F("d down | u up | c cycle | 1/2 down time | 3/4 up time | 5/6 up speed | 7/8 down speed | p print"));
  printSettings();
}

void loop() {
  if (!Serial.available()) {
    return;
  }
  char c = Serial.read();
  if (c == 'd') {
    moveDown();
  } else if (c == 'u') {
    moveUp();
  } else if (c == 'c') {
    moveDown();
    delay(1000);
    moveUp();
  } else if (c == '1' || c == '2') {
    downMs = constrain(downMs + (c == '1' ? -10 : 10), 10, 2000);
    printSettings();
  } else if (c == '3' || c == '4') {
    upMs = constrain(upMs + (c == '3' ? -100 : 100), 100, 5000);
    printSettings();
  } else if (c == '5' || c == '6') {
    upSpeed = constrain(upSpeed + (c == '5' ? -10 : 10), 20, 400);
    printSettings();
  } else if (c == '7' || c == '8') {
    downSpeed = constrain(downSpeed + (c == '7' ? -10 : 10), 20, 400);
    printSettings();
  } else if (c == 'p') {
    printSettings();
  }
}

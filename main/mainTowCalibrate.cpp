// Tow servo test for the FS90R continuous servo on D11. Simple keys, one action each.
// Copy this file over src/main.cpp, upload, open the Serial Monitor at 9600 and keep USB plugged in.
// The servo needs the battery (6 V servo rail); unplug the battery at any time to stop it.
// The tracks are held off. The servo goes limp after every action.
//
// Type a number and press Enter:
//   1  tiny turn DOWN   (20 ms, one servo pulse)
//   2  tiny turn UP     (20 ms)
//   3  bigger turn DOWN (60 ms)
//   4  bigger turn UP   (60 ms)
//   0  "I'm at the top now": resets the counter to zero
// The log keeps a running total of how long it has turned since the last 0, so you can read off
// how long it takes from the top down to the tow.
#include <Arduino.h>
#include <Servo.h>

const uint8_t TOW_PIN = 11;
const int STOP_US = 1500;         // the servo stands still at this pulse
const int SPEED_US = 60;          // how far from STOP_US it turns (bigger = faster; 100 was too aggressive, 50 with short bursts didnt move)
const unsigned long SMALL_MS = 20;    // one servo pulse; the full travel took under 100 ms
const unsigned long LONG_MS = 60;

// Track speed pins, only used to keep the motors off.
const uint8_t LEFT_EN = 6;
const uint8_t RIGHT_EN = 5;

Servo tow;
long totalMs = 0;   // time turned down (keys 1/3) minus up (keys 2/4) since the last 0

// Turns down (direction 1, pulse below STOP_US) or up (-1) for ms, then stops and goes limp.
void turn(int8_t direction, unsigned long ms) {
  tow.writeMicroseconds(STOP_US - direction * SPEED_US);
  tow.attach(TOW_PIN);
  delay(ms);
  tow.writeMicroseconds(STOP_US);
  delay(40);
  tow.detach();
  totalMs += direction * (long)ms;

  Serial.print(direction > 0 ? F("Down ") : F("Up "));
  Serial.print(ms);
  Serial.print(F(" ms  |  total from zero: "));
  Serial.print(totalMs);
  Serial.println(F(" ms down"));
}

void setup() {
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
  Serial.begin(9600);
  Serial.println(F("FS90R tow test (D11)"));
  Serial.println(F("1 small DOWN | 2 small UP | 3 long DOWN | 4 long UP | 0 here is the top"));
}

void loop() {
  if (!Serial.available()) {
    return;
  }
  char c = Serial.read();
  if (c == '1') {
    turn(1, SMALL_MS);
  } else if (c == '2') {
    turn(-1, SMALL_MS);
  } else if (c == '3') {
    turn(1, LONG_MS);
  } else if (c == '4') {
    turn(-1, LONG_MS);
  } else if (c == '0') {
    totalMs = 0;
    Serial.println(F("Counter reset: this is zero (the top)"));
  }
}

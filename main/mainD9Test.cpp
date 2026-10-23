// D9 test bench. Copy this file over src/main.cpp, upload, and open the serial monitor at 9600 baud.
// Type a menu number + Enter to run a test; press Enter again to stop it.
#include "Generic/GenericPlatform/GenericPlatform.h"
#include <Arduino.h>
#include "D9/D9/D9.h"
#include "D9/D9Test/D9Test.h"

// D9 wiring and settings, copied from mainD9.cpp. Keep the two files in sync when the wiring changes.
const uint8_t LEFT_IN1 = 8, LEFT_IN2 = 7, LEFT_EN = 6;  // 7/8 swapped: the D9 drove backwards
const uint8_t RIGHT_IN1 = 8, RIGHT_IN2 = 7, RIGHT_EN = 5;
const uint8_t LEFT_IR = A5, RIGHT_IR = A4;
const uint16_t LINE_THRESHOLD = 300;
const uint8_t LORA_TXD = 2, LORA_RXD = 10;  // module TXD -> D2, module RXD <- D10
const char PLATFORM_NAME[] = "D9";
const uint8_t SONIC_TRIG = NO_PIN, SONIC_ECHO = NO_PIN;  // the D9 has no ultrasonic
const uint8_t IR_RECEIVER = 4;
const uint8_t LDR_PIN = A3;
const uint16_t LASER_THRESHOLD = 300;
const uint8_t SERVO_PIN = 3;   // radar servo; the front lever is on D9 (no code yet)
const uint8_t TOW_PIN = 11;    // tow hook (FS90R continuous servo)

GenericPlatform basePlatform(
  LEFT_IN1, LEFT_IN2, LEFT_EN,
  RIGHT_IN1, RIGHT_IN2, RIGHT_EN,
  LEFT_IR, RIGHT_IR, LORA_TXD, LORA_RXD,
  SONIC_TRIG, SONIC_ECHO, IR_RECEIVER,
  LINE_THRESHOLD, PLATFORM_NAME
);

D9Platform d9(basePlatform, LDR_PIN, LASER_THRESHOLD, SERVO_PIN, TOW_PIN);

D9Test tester(
    d9,
    LEFT_IN1, LEFT_IN2, LEFT_EN,
    RIGHT_IN1, RIGHT_IN2, RIGHT_EN,
    LEFT_IR, RIGHT_IR, LINE_THRESHOLD,
    LORA_TXD, LORA_RXD, PLATFORM_NAME,
    SONIC_TRIG, SONIC_ECHO,
    IR_RECEIVER,
    LDR_PIN, LASER_THRESHOLD,
    SERVO_PIN
);

// Starts serial diagnostics and every subsystem, then shows the test menu.
void setup() {
    Serial.begin(9600);
    tester.begin();
}

// Waits for a menu command and runs the chosen test.
void loop() {
    tester.update();
}

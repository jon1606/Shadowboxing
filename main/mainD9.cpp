// D9 mission. Copy this file over src/main.cpp, upload, unplug USB and run it on the battery.
// It runs on its own from power-on until the battery is disconnected; there is no HAMAL.
//
//   1. Power on: POWER_SETTLE_MS for the battery connection to settle, then a START_DELAY_MS countdown.
//   2. Follow the line with the radar sweeping all the time.
//   3. When the laser hits the LDR: stop, point the radar at it for THREAT_HOLD_MS, send THREAT
//      (distance, radar angle) to ALL, then drive on. The laser has to go away before it counts again.
//   4. At IR markers 2, 11, 13 and 14: stop and sweep the radar SCAN_PASSES times, then drive on.
//   5. At IR marker 16: tow (hook down, retrace the path backwards for 7 s, hook up), then drive on.
//   Every other IR code is ignored, and each marker is handled once per run. Each marker it acts on
//   is also sent to ALL as IR (x = the code), for debugging.
//
// Board LED (D13):
//   3 quick flashes   = a reset cut the last tow short; the hook was lifted and there's no tow this run
//   slow blink (1 s)  = start countdown, put the D9 on the line
//   on                = following the line
//   blink (0.5 s)     = stopped at a marker, scanning
//   off               = stopped for a threat, or towing
// With USB plugged in, the Serial Monitor (9600) logs every step.
#include <Arduino.h>
#include "Generic/GenericPlatform/GenericPlatform.h"
#include "D9/D9/D9.h"

GenericPlatform basePlatform(
    8, 7, 6,        // Left track: IN1, IN2, EN (IN1/IN2 swapped so positive speed drives forward)
    8, 7, 5,        // Right track: IN1, IN2, EN (same direction pins as the left track)
    A5, A4,         // Line sensors: left, right
    2, 10,          // LoRa: module TXD -> D2, module RXD <- D10
    NO_PIN, NO_PIN, // No ultrasonic sensor
    4,              // IR receiver
    300, "D9"       // Line threshold, LoRa name
);

D9Platform d9(
    basePlatform,
    A3,  // LDR
    300, // Laser threshold: a reading below it means the laser is on the LDR
    3,   // Radar servo (270 deg)
    11   // Tow hook servo (FS90R continuous)
);

// What the D9 does at an IR marker.
enum MarkerAction : uint8_t {
    SCAN,
    TOW
};

struct Marker {
    uint8_t code;
    MarkerAction action;
    bool handled;   // set the first time it is read: each marker is handled once per run
};

// The markers the D9 acts on; every other IR code is ignored.
Marker markers[] = {
    {2,  SCAN, false},
    {11, SCAN, false},
    {13, SCAN, false},
    {14, SCAN, false},
    {16, TOW,  false},
};

const unsigned long POWER_SETTLE_MS = 1000;
const unsigned long START_DELAY_MS = 5000;
const unsigned long THREAT_HOLD_MS = 3000;
const uint8_t SCAN_PASSES = 2;              // full radar passes at a SCAN marker

// What the D9 is doing right now.
enum Phase {
    DRIVING,
    SCANNING,
    THREAT_HOLD
};

Phase phase = DRIVING;
unsigned long phaseStart = 0;
uint16_t threats = 0;

// Prints "[seconds] " before a log line.
void logTime() {
    Serial.print('[');
    Serial.print(millis() / 1000.0f, 1);
    Serial.print(F("s] "));
}

// Sends a message to every station without waiting for an answer.
void broadcast(const char* type, int x, int angle) {
    basePlatform.sendMessage("ALL", type, x, angle, false);
}

// Blinks the board LED, on for halfPeriodMs and off for halfPeriodMs.
void blinkLed(unsigned long halfPeriodMs) {
    digitalWrite(LED_BUILTIN, (millis() / halfPeriodMs) % 2);
}

void flashLed(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(150);
        digitalWrite(LED_BUILTIN, LOW);
        delay(150);
    }
}

void enterPhase(Phase next) {
    phase = next;
    phaseStart = millis();
}

// Returns the marker the IR receiver just read if it hasn't been handled yet this run, else nullptr.
Marker* readNewMarker() {
    uint8_t code = basePlatform.updateIRReceiver();
    for (Marker& marker : markers) {
        if (marker.code == code && !marker.handled) {
            marker.handled = true;
            return &marker;
        }
    }
    return nullptr;
}

// Marks every tow marker as handled, so there's no tow this run.
void skipTows() {
    for (Marker& marker : markers) {
        if (marker.action == TOW) {
            marker.handled = true;
        }
    }
}

// Follows the line with the radar sweeping from its start angle.
void startDriving() {
    d9.restartRadar();
    d9.startDriving();
    logTime();
    Serial.println(F("Following the line"));
    enterPhase(DRIVING);
}

// Stops, keeps the radar on the laser and reports the threat to everyone.
void stopForThreat() {
    d9.stopDriving();
    d9.holdRadar();
    threats++;
    logTime();
    Serial.print(F("THREAT #"));
    Serial.print(threats);
    Serial.print(F(" at "));
    Serial.print(d9.getRadarAngle());
    Serial.println(F(" deg"));
    broadcast("THREAT", (int)basePlatform.totalDistance(), d9.getRadarAngle());
    enterPhase(THREAT_HOLD);
}

// Scans or tows at a marker read for the first time.
void stopAtMarker(const Marker& marker) {
    d9.stopDriving();
    broadcast("IR", marker.code, 0);
    logTime();
    Serial.print(F("IR marker "));
    Serial.print(marker.code);
    if (marker.action == TOW) {
        Serial.println(F(": towing"));
        digitalWrite(LED_BUILTIN, LOW);
        d9.tow();
        startDriving();
    } else {
        Serial.println(F(": scanning"));
        d9.restartRadar();
        enterPhase(SCANNING);
    }
}

void drive() {
    digitalWrite(LED_BUILTIN, HIGH);
    d9.followLine();
    d9.sweepRadar();
    if (d9.threatDetected()) {
        stopForThreat();
        return;
    }
    Marker* marker = readNewMarker();
    if (marker != nullptr) {
        stopAtMarker(*marker);
    }
}

void scan() {
    blinkLed(250);
    d9.sweepRadar();
    if (d9.threatDetected()) {
        stopForThreat();
    } else if (d9.getRadarPasses() >= SCAN_PASSES) {
        logTime();
        Serial.println(F("No threat here"));
        startDriving();
    }
}

void holdOnThreat() {
    digitalWrite(LED_BUILTIN, LOW);
    if (millis() - phaseStart >= THREAT_HOLD_MS) {
        startDriving();
    }
}

void setup() {
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println(F("D9 mission: booting"));
    delay(POWER_SETTLE_MS);
    d9.begin();
    if (d9.towWasInterrupted()) {
        Serial.println(F("A reset cut the last tow short: hook lifted, no tow this run"));
        skipTows();
        flashLed(3);
    }

    Serial.println(F("Starting in 5 s"));
    unsigned long start = millis();
    while (millis() - start < START_DELAY_MS) {
        blinkLed(500);
    }
    startDriving();
}

void loop() {
    switch (phase) {
        case DRIVING:
            drive();
            break;
        case SCANNING:
            scan();
            break;
        case THREAT_HOLD:
            holdOnThreat();
            break;
    }
}

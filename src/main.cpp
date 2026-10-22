// D9 field demo: line following + threat scanning, no computer or HAMAL needed.
// Copy this file over src/main.cpp, upload, unplug USB and run it on the battery.
//
// While following the line the radar keeps sweeping 0-180 deg. At an IR marker the D9 stops and
// sweeps MARKER_SWEEPS full passes, then drives on. Whenever the laser hits the LDR (driving or
// stopped), the D9 stops, points the radar at it for THREAT_HOLD_MS, then follows the line again.
// It never stops for a lost line.
//
// Line following = updateDrive()'s steering plus turn memory: after a long correction (a curve),
// if both sensors go dark it keeps turning the same way for up to TURN_HOLD_MS, so a sharp turn
// that slips past a sensor gets caught again instead of the D9 driving straight off it.
//
// HAMAL link (LoRa, this D9 is "D9"):
//   waits for GO before moving; STOP stops everything for good (until reset);
//   THREAT (x = distance, angle) goes to HML on every laser hit, waiting for its ACK while stopped;
//   POSITION goes to HML every 5 s while driving and at each marker (no waiting).
//
// Board LED (D13):
//   slow blink (1 s)  = waiting for GO from the HAMAL, put the D9 on the line
//   on                = following the line (radar sweeping)
//   blink (0.5 s)     = stopped at a marker, sweeping
//   off               = threat found, the radar points at it
//   fast blink        = STOP received, everything stopped
// With USB plugged in, the Serial Monitor (9600) logs every step.
#include "Generic/GenericPlatform/GenericPlatform.h"
#include <Arduino.h>
#include "D9/D9/D9.h"
#include "Generic/Motor/Motor.h"
#include "Generic/TankDrive/TankDrive.h"

// Same wiring as mainD9.cpp.
GenericPlatform basePlatform(
  8, 7, 6, //Left motor (7/8 swapped: the D9 drove backwards)
  8, 7, 5, //Right motor
  A5, A4, 2, 10, //Ir analog and Lora
  NO_PIN, NO_PIN, 4, //No ultrasonic, IR receiver
  300 , "D9" //Threshold and LoRa name
);

D9Platform d9(
    basePlatform,
    A3, // LDR pin
    300, // Laser threshold
    3 // Radar servo pin
);

// Direct track control for the demo's own steering (same pins as basePlatform's motors).
Motor leftMotor(8, 7, 6);
Motor rightMotor(8, 7, 5);
TankDrive tracks(leftMotor, rightMotor);

const uint8_t LEFT_LINE_PIN = A5;
const uint8_t RIGHT_LINE_PIN = A4;
const int LINE_THRESHOLD = 300;
const uint8_t LED = LED_BUILTIN;

// Radar sweep limits, as set in D9.cpp.
const uint8_t RADAR_MIN = 0;
const uint8_t RADAR_MAX = 180;

// Value updateIRReceiver() returns when no marker was seen.
const int8_t NO_MARKER = 100;

// The route's IR markers (northLoop()/southLoop() in mainD9.cpp); any other code is noise.
const int8_t ROUTE_MARKERS[] = {2, 4, 11, 13, 16};
const unsigned long MARKER_CONFIRM_MS = 300;    // the same marker must show up twice within this
const uint8_t THREAT_CONFIRM_READS = 3;         // LDR readings in a row under the threshold

// Line following, same speeds as GenericPlatform.cpp.
const int BASE_PWM = 200;
const int TURN_PWM = 0;                         // inner track while steering
const unsigned long CURVE_MS = 120;             // a correction this long counts as a curve
const unsigned long TURN_HOLD_MS = 0;           // keep turning this long after a curve's line slips away (0 = off; 1500 overshot on straights)

const uint8_t MARKER_SWEEPS = 2;                // full 0-180 passes at each marker
const unsigned long MARKER_COOLDOWN_MS = 3000;  // ignore the same marker this long after leaving it
const unsigned long THREAT_HOLD_MS = 3000;
const unsigned long CLEAR_MS = 500;             // laser must be gone this long before it counts again

enum Phase { DRIVING, MARKER_SCAN, THREAT, STOPPED };

Phase phase = DRIVING;
unsigned long phaseStart = 0;
int8_t turnDirection = 0;        // 1 steering right, -1 steering left, 0 straight
unsigned long correctionStart = 0;
unsigned long holdUntil = 0;     // keep turning until this time after a curve
uint8_t sweepsDone = 0;
uint8_t lastEnd = RADAR_MIN;     // which end of the sweep the radar touched last
int8_t lastMarker = NO_MARKER;
unsigned long markerLeftAt = 0;
bool laserArmed = true;          // false right after a threat, until the laser has gone away
uint8_t laserReads = 0;          // readings in a row under the threshold
int8_t markerCandidate = NO_MARKER;
unsigned long markerCandidateAt = 0;
unsigned long laserClearSince = 0;
uint16_t threats = 0;

// Prints "[seconds] " before a log line.
void logTime() {
  Serial.print('[');
  Serial.print(millis() / 1000.0f, 1);
  Serial.print(F("s] "));
}

void enterPhase(Phase next) {
  phase = next;
  phaseStart = millis();
}

// Steers the tracks like updateDrive(), plus turn memory for sharp curves.
void followLine() {
  bool left = analogRead(LEFT_LINE_PIN) > LINE_THRESHOLD;
  bool right = analogRead(RIGHT_LINE_PIN) > LINE_THRESHOLD;
  unsigned long now = millis();

  if (left != right) {
    // One sensor is on the line's edge: steer toward the other one, as updateDrive() does.
    int8_t wanted = left ? 1 : -1;
    if (wanted != turnDirection) {
      turnDirection = wanted;
      correctionStart = now;
      holdUntil = 0;
    }
    // While a curve keeps going, keep extending how long to hold the turn if the line slips away.
    if (now - correctionStart >= CURVE_MS) {
      holdUntil = now + TURN_HOLD_MS;
    }
  } else if (left && right) {
    // Both dark: either centered again, or the line slipped out past a sensor on a sharp turn.
    if (turnDirection != 0 && (long)(holdUntil - now) <= 0) {
      turnDirection = 0;
    }
  }
  // Neither sensor dark: keep the last command, as updateDrive() does.

  if (turnDirection > 0) {
    tracks.drive(BASE_PWM, TURN_PWM);
  } else if (turnDirection < 0) {
    tracks.drive(TURN_PWM, BASE_PWM);
  } else {
    tracks.drive(BASE_PWM, BASE_PWM);
  }
}

// Restarts the radar sweep from 0 deg and starts counting full passes again.
void restartRadar() {
  d9.getServoScanner().begin();
  sweepsDone = 0;
  lastEnd = RADAR_MIN;
}

// Steps the radar and counts a pass each time it reaches the opposite end.
void sweepRadar() {
  ServoScanner& radar = d9.getServoScanner();
  radar.update();
  uint8_t angle = radar.getAngle();
  if ((angle >= RADAR_MAX && lastEnd == RADAR_MIN) || (angle <= RADAR_MIN && lastEnd == RADAR_MAX)) {
    lastEnd = angle >= RADAR_MAX ? RADAR_MAX : RADAR_MIN;
    sweepsDone++;
  }
}

// True once per laser hit, after THREAT_CONFIRM_READS readings in a row under the threshold.
// After a threat, the laser has to leave the LDR for CLEAR_MS before it counts again.
bool laserHit() {
  bool detected = d9.getEnemyDetector().detected();
  laserReads = detected ? laserReads + 1 : 0;
  if (!laserArmed) {
    if (detected) {
      laserClearSince = millis();
    } else if (millis() - laserClearSince >= CLEAR_MS) {
      laserArmed = true;
    }
    return false;
  }
  return laserReads >= THREAT_CONFIRM_READS;
}

// Returns a route marker once it has been seen twice within MARKER_CONFIRM_MS, else NO_MARKER.
int8_t confirmedMarker() {
  int8_t code = basePlatform.updateIRReceiver();
  bool onRoute = false;
  for (uint8_t i = 0; i < sizeof(ROUTE_MARKERS); i++) {
    onRoute = onRoute || code == ROUTE_MARKERS[i];
  }
  if (!onRoute) {
    return NO_MARKER;
  }
  unsigned long now = millis();
  if (code == markerCandidate && now - markerCandidateAt <= MARKER_CONFIRM_MS) {
    markerCandidate = NO_MARKER;
    return code;
  }
  markerCandidate = code;
  markerCandidateAt = now;
  return NO_MARKER;
}

// Resumes line following. The turn memory is kept, so a stop in the middle of a curve
// carries on turning the same way.
void startDriving() {
  restartRadar();
  logTime();
  Serial.println(F("Following the line, radar sweeping"));
  enterPhase(DRIVING);
}

void threatFound() {
  tracks.stop();
  ServoScanner& radar = d9.getServoScanner();
  radar.stop();
  laserArmed = false;
  laserClearSince = millis();
  threats++;
  logTime();
  Serial.print(F("THREAT #"));
  Serial.print(threats);
  Serial.print(F(" at "));
  Serial.print(radar.getAngle());
  Serial.print(F(" deg, ldr "));
  Serial.println(d9.getEnemyDetector().getLastReading());
  // The tracks are stopped, so waiting for the ACK (with resends) is safe here.
  bool acked = basePlatform.sendMessage("HML", "THREAT", (int)basePlatform.totalDistance(), radar.getAngle());
  logTime();
  Serial.println(acked ? F("THREAT ACKed by HML") : F("THREAT not ACKed"));
  enterPhase(THREAT);
}

// Logs every message the radio hands over, so missed or garbled commands show up in the log.
void logReceived(const String& message) {
  if (message.length() == 0) {
    return;
  }
  logTime();
  Serial.print(F("LoRa rx: "));
  Serial.println(message);
}

// Checks the radio; on STOP, stops the tracks and radar for good. Returns true if stopped.
bool stopRequested() {
  logReceived(basePlatform.listen());
  if (!basePlatform.needToStop()) {
    return false;
  }
  if (phase != STOPPED) {
    tracks.stop();
    d9.getServoScanner().stop();
    logTime();
    Serial.println(F("STOP from HAMAL, everything stopped"));
    enterPhase(STOPPED);
  }
  return true;
}

void setup() {
  Serial.begin(9600);
  d9.begin();
  tracks.begin();
  pinMode(LED, OUTPUT);
  Serial.println(F("D9 field demo: waiting for GO from the HAMAL"));
  while (true) {
    digitalWrite(LED, (millis() / 500) % 2);
    String message = basePlatform.listen();
    logReceived(message);
    if (basePlatform.needToStop()) {
      stopRequested();
      return;
    }
    if (message.startsWith("GO")) {
      break;
    }
  }
  logTime();
  Serial.println(F("GO received"));
  startDriving();
}

void loop() {
  unsigned long now = millis();
  if (stopRequested()) {
    digitalWrite(LED, (now / 100) % 2);
    return;
  }

  if (phase == DRIVING) {
    digitalWrite(LED, HIGH);
    followLine();
    sweepRadar();
    // Fire and forget, every 5 s (GenericPlatform's LOC_INTERVAL).
    basePlatform.sendLocationIfDue();
    if (laserHit()) {
      threatFound();
      return;
    }

    int8_t marker = confirmedMarker();
    bool sameMarkerTooSoon = marker == lastMarker && now - markerLeftAt < MARKER_COOLDOWN_MS;
    if (marker != NO_MARKER && !sameMarkerTooSoon) {
      tracks.stop();
      lastMarker = marker;
      restartRadar();
      logTime();
      Serial.print(F("IR marker "));
      Serial.print((uint8_t)marker);
      Serial.println(F(", stopped to sweep"));
      // updateIRReceiver() already set the distance to this marker's calibrated value.
      basePlatform.sendMessage("HML", "POSITION", (int)basePlatform.totalDistance(), 0, false);
      enterPhase(MARKER_SCAN);
    }
  } else if (phase == MARKER_SCAN) {
    digitalWrite(LED, (now / 250) % 2);
    sweepRadar();
    if (laserHit()) {
      threatFound();
    } else if (sweepsDone >= MARKER_SWEEPS) {
      logTime();
      Serial.println(F("No threat at this marker"));
      markerLeftAt = now;
      startDriving();
    }
  } else {
    digitalWrite(LED, LOW);
    if (now - phaseStart >= THREAT_HOLD_MS) {
      markerLeftAt = now;
      startDriving();
    }
  }
}

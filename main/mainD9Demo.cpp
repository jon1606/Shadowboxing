// D9 field demo: runs the whole scenario on its own, no HAMAL or computer needed.
// Copy this file over src/main.cpp, upload, unplug USB and run it on the battery.
//
// Scenario:
//   1. Power on: 1 s for the power to settle, then a 5 s countdown. The tow is lifted at the end of
//      every tow, so it starts lifted (set TOW_LIFT_AT_BOOT to also lift it at power-on).
//   2. Follow the line with the radar sweeping 50-245 deg. Whenever the laser hits the LDR, stop,
//      point the radar at it for THREAT_HOLD_MS and broadcast THREAT to ALL over LoRa (no waiting),
//      then keep going. It never stops for a lost line.
//   3. At each IR point (any code; 12 is ignored everywhere): stop and sweep the radar
//      MARKER_SWEEPS full passes, then drive on. Each IR code is handled only the first time it is
//      read; after that it is ignored for the rest of the run.
//   IR debug messages to ALL (x = the code): IR when an IR point is read for the first time and acted
//   on, IGNORED when a code that was already recorded this run is read again (once per IR_REPORT_MS
//   per code).
//   4. At the tow point (TOW_MARKER): stop, lower the tow, drive backwards for REVERSE_MS, stop,
//      lift the tow, then follow the line again.
//
// Every boot (power-on, reset button, upload) starts a fresh run: no IR point is handled yet.
// Only an unfinished tow is remembered across a reset (e.g. a battery dip while reversing with the
// load): the LED flashes 3 times, the tow is lifted and the tow point is skipped, so it can't loop.
//
// Reversing retraces the path: while following the line, the D9 records its track speeds every
// RECORD_MS for the last REVERSE_MS; at the tow point it plays them back newest first with both tracks
// reversed, which drives the same curves backwards. The line sensors sit at the front, so steering
// by them in reverse is unstable on its own; REVERSE_TRIM_PWM (0 = off) adds a gentle correction
// on top of the replay.
//
// Board LED (D13):
//   3 quick flashes   = a reset interrupted the tow; the tow point is skipped this run
//   slow blink (1 s)  = start countdown, put the D9 on the line
//   on                = following the line (radar sweeping)
//   blink (0.5 s)     = stopped at an IR point, sweeping
//   off               = threat found, the radar points at it
//   fast blink        = towing
// With USB plugged in, the Serial Monitor (9600) logs every step.
#include "Generic/GenericPlatform/GenericPlatform.h"
#include <Arduino.h>
#include <Servo.h>
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

// Tow: FS90R continuous servo, moved by time (calibrated with mainTowTest.cpp).
Servo tow;
const uint8_t TOW_PIN = 11;
const int TOW_STOP_US = 1500;
const int TOW_SPEED_US = 60;                 // down = stop - speed, up = stop + speed
const unsigned long TOW_DOWN_MS = 300;
const unsigned long TOW_UP_MS = 400;

const uint8_t LEFT_LINE_PIN = A5;
const uint8_t RIGHT_LINE_PIN = A4;
const int LINE_THRESHOLD = 300;
const uint8_t LED = LED_BUILTIN;

// Value updateIRReceiver() returns when no marker was seen.
const int8_t NO_MARKER = 100;
const int8_t TOW_MARKER = 16;
const uint8_t MARKER_SWEEPS = 2;                // full radar passes at each IR point
const unsigned long IR_REPORT_MS = 2000;        // the same IR code is broadcast at most this often

// Radar sweep limits, as set in D9.cpp.
const uint8_t RADAR_MIN = 50;
const uint8_t RADAR_MAX = 245;
const uint8_t THREAT_CONFIRM_READS = 3;         // LDR readings in a row under the threshold

// Line following, same speeds as GenericPlatform.cpp.
const int BASE_PWM = 200;
const int TURN_PWM = 0;                         // inner track once a correction lasts (hard turn)
const int TURN_OUTER_PWM = 255;                 // outer track in a hard turn: skidding round a stopped
                                                // track needs force, at 200 it stalled mid-turn
const int SOFT_TURN_PWM = 120;                  // inner track at the start of a correction (gentle turn)
const unsigned long SOFT_TURN_MS = 150;         // how long a correction stays gentle (0 = always hard)
const unsigned long CURVE_MS = 120;             // a correction this long counts as a curve
const unsigned long TURN_HOLD_MS = 0;           // keep turning this long after a curve's line slips away (0 = off; 1500 overshot on straights)

const int KICK_PWM = 255;                       // short full-power push when starting from a stop
const unsigned long KICK_MS = 200;              // (0 = no kick)
const unsigned long POWER_SETTLE_MS = 1000;     // let the battery connection settle before anything moves
const unsigned long START_DELAY_MS = 5000;
// Lifting the tow at power-on pushes it against its top stop. With the tow servo powered from the
// Arduino's 5 V, that stall current resets the Arduino and it never starts driving. Only turn this
// on once the tow is powered from the 6 V servo rail.
const bool TOW_LIFT_AT_BOOT = false;
const unsigned long THREAT_HOLD_MS = 3000;
const unsigned long CLEAR_MS = 500;             // laser must be gone this long before it counts again
const int REVERSE_PWM = 200;                    // track speed while reversing past the recorded path
const unsigned long REVERSE_MS = 7000;
const unsigned long RECORD_MS = 50;             // how often the track speeds are recorded
const uint8_t RECORD_SIZE = REVERSE_MS / RECORD_MS;
const int REVERSE_TRIM_PWM = 0;                 // sensor correction while reversing (0 = off, try 30-60)

enum Phase { DRIVING, MARKER_SCAN, THREAT };

Phase phase = DRIVING;
unsigned long phaseStart = 0;
int8_t turnDirection = 0;        // 1 steering right, -1 steering left, 0 straight
unsigned long correctionStart = 0;
unsigned long holdUntil = 0;     // keep turning until this time after a curve
bool laserArmed = true;          // false right after a threat, until the laser has gone away
uint8_t laserReads = 0;          // readings in a row under the threshold
unsigned long laserClearSince = 0;
uint8_t sweepsDone = 0;
uint8_t lastEnd = RADAR_MIN;     // which end of the sweep the radar touched last
uint16_t threats = 0;
int8_t lastReportedCode = NO_MARKER;
unsigned long lastReportAt = 0;
bool markerRecorded[256];        // one flag per IR code: true once it was read this run; cleared every boot

// Tow progress kept in .noinit RAM: a reset leaves it alone, only a power cut loses it. The magic
// number and checksum tell a real saved state from the random contents RAM has after power-on.
struct RunState {
  uint32_t magic;
  bool towInProgress;    // set from the tow's start until the tow is lifted again
  bool towIsDown;
  uint8_t check;
};
const uint32_t RUN_MAGIC = 0xD9C0FFEEUL;
RunState runState __attribute__((section(".noinit")));

uint8_t runStateCheck() {
  uint8_t sum = 0x5A;
  const uint8_t* bytes = (const uint8_t*)&runState;
  for (uint8_t i = 0; i < sizeof(RunState) - 1; i++) {
    sum = (sum << 1 | sum >> 7) ^ bytes[i];
  }
  return sum;
}

void saveRunState() {
  runState.check = runStateCheck();
}

// True if RAM still holds this run's memory, i.e. the board was reset without a power cut.
bool runStateSurvived() {
  return runState.magic == RUN_MAGIC && runState.check == runStateCheck();
}

// Track speeds of the last REVERSE_MS of driving, oldest overwritten first.
uint8_t recordLeft[RECORD_SIZE];
uint8_t recordRight[RECORD_SIZE];
uint8_t recordHead = 0;          // next slot to write; the newest sample is just before it
uint8_t recordCount = 0;
unsigned long lastRecord = 0;
int driveLeft = 0;               // track speeds followLine() set last
int driveRight = 0;

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

// Turns the tow servo at speedUs from the stop pulse for ms (negative speed = down), then
// stops it and lets it go limp.
void turnTow(int speedUs, unsigned long ms) {
  tow.writeMicroseconds(TOW_STOP_US + speedUs);
  tow.attach(TOW_PIN);
  delay(ms);
  tow.writeMicroseconds(TOW_STOP_US);
  delay(40);
  tow.detach();
}

void towUp() {
  logTime();
  Serial.println(F("Tow up"));
  turnTow(TOW_SPEED_US, TOW_UP_MS);
  runState.towIsDown = false;
  saveRunState();
}

void towDown() {
  logTime();
  Serial.println(F("Tow down"));
  runState.towIsDown = true;
  saveRunState();
  turnTow(-TOW_SPEED_US, TOW_DOWN_MS);
}

// Drives forward and remembers the speeds for recording.
void driveForward(int left, int right) {
  driveLeft = left;
  driveRight = right;
  tracks.drive(left, right);
}

// Stores the current track speeds every RECORD_MS while following the line.
void recordDriving() {
  unsigned long now = millis();
  if (now - lastRecord < RECORD_MS) {
    return;
  }
  lastRecord = now;
  recordLeft[recordHead] = driveLeft;
  recordRight[recordHead] = driveRight;
  recordHead = (recordHead + 1) % RECORD_SIZE;
  if (recordCount < RECORD_SIZE) {
    recordCount++;
  }
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

  // Small drifts get a gentle correction; one that lasts (a real curve) turns hard.
  bool hard = now - correctionStart >= SOFT_TURN_MS;
  int inner = hard ? TURN_PWM : SOFT_TURN_PWM;
  int outer = hard ? TURN_OUTER_PWM : BASE_PWM;
  if (turnDirection > 0) {
    driveForward(outer, inner);
  } else if (turnDirection < 0) {
    driveForward(inner, outer);
  } else {
    driveForward(BASE_PWM, BASE_PWM);
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

// Sends an IR debug message to ALL (fire and forget). IGNORED repeats at most every IR_REPORT_MS
// per code; IR (a new point) is always sent.
void reportIrCode(const char* type, int8_t code) {
  unsigned long now = millis();
  bool always = type[0] == 'I' && type[1] == 'R';
  if (!always && code == lastReportedCode && now - lastReportAt < IR_REPORT_MS) {
    return;
  }
  lastReportedCode = code;
  lastReportAt = now;
  logTime();
  Serial.print(type);
  Serial.print(F(" "));
  Serial.println((uint8_t)code);
  basePlatform.sendMessage("ALL", type, (uint8_t)code, 0, false);
}

// Returns an IR code the first time it is read this run and records it; a code that was
// already recorded returns NO_MARKER.
int8_t newMarker() {
  int8_t code = basePlatform.updateIRReceiver();
  if (code == NO_MARKER) {
    return NO_MARKER;
  }
  if (markerRecorded[(uint8_t)code]) {
    reportIrCode("IGNORED", code);
    return NO_MARKER;
  }
  markerRecorded[(uint8_t)code] = true;
  reportIrCode("IR", code);
  return code;
}

// Restarts the radar sweep from its start angle and starts counting full passes again.
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

void startDriving() {
  restartRadar();
  // From standstill the tracks may not have the force to break free (especially in a pivot),
  // so start with a short straight kick at full power.
  tracks.drive(KICK_PWM, KICK_PWM);
  delay(KICK_MS);
  // Time spent stopped isn't part of the path, so recording restarts from now.
  lastRecord = millis();
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
  // Broadcast, fire and forget: nobody has to answer.
  basePlatform.sendMessage("ALL", "THREAT", (int)basePlatform.totalDistance(), radar.getAngle(), false);
  enterPhase(THREAT);
}

// Replays the recorded track speeds newest first with both tracks reversed, for REVERSE_MS.
// If less than REVERSE_MS was recorded, the rest is driven straight back.
void reverseAlongPath() {
  logTime();
  Serial.print(F("Reversing 7 s along the recorded path ("));
  Serial.print(recordCount * RECORD_MS);
  Serial.println(F(" ms recorded)"));
  uint8_t index = recordHead;
  uint8_t remaining = recordCount;
  unsigned long start = millis();
  while (millis() - start < REVERSE_MS) {
    int left = REVERSE_PWM;
    int right = REVERSE_PWM;
    if (remaining > 0) {
      index = (index + RECORD_SIZE - 1) % RECORD_SIZE;
      left = recordLeft[index];
      right = recordRight[index];
      remaining--;
    }
    if (REVERSE_TRIM_PWM > 0) {
      // Same decision as forward; in reverse the correction slows the track that was fast.
      bool leftDark = analogRead(LEFT_LINE_PIN) > LINE_THRESHOLD;
      bool rightDark = analogRead(RIGHT_LINE_PIN) > LINE_THRESHOLD;
      if (leftDark && !rightDark) {
        left = max(0, left - REVERSE_TRIM_PWM);
      } else if (rightDark && !leftDark) {
        right = max(0, right - REVERSE_TRIM_PWM);
      }
    }
    tracks.drive(-left, -right);
    unsigned long stepStart = millis();
    while (millis() - stepStart < RECORD_MS) {
      digitalWrite(LED, (millis() / 100) % 2);
    }
  }
  tracks.stop();
}

// IR point: stop and sweep the radar MARKER_SWEEPS passes before driving on.
void startMarkerScan(int8_t marker) {
  tracks.stop();
  restartRadar();
  logTime();
  Serial.print(F("IR point "));
  Serial.print((uint8_t)marker);
  Serial.println(F(": stopped to scan"));
  enterPhase(MARKER_SCAN);
}

// Tow point: lower the tow, drive backwards with it, stop, lift it, then drive on.
void runTow(int8_t marker) {
  tracks.stop();
  d9.getServoScanner().stop();
  digitalWrite(LED, HIGH);
  logTime();
  Serial.print(F("IR point "));
  Serial.print((uint8_t)marker);
  Serial.println(F(": towing"));
  runState.towInProgress = true;
  saveRunState();
  towDown();
  delay(500);
  reverseAlongPath();
  delay(300);
  towUp();
  runState.towInProgress = false;
  saveRunState();
  // The recorded path was used up driving back.
  recordHead = 0;
  recordCount = 0;
  startDriving();
}

void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  tracks.begin();
  Serial.println(F("D9 field demo: booting"));
  delay(POWER_SETTLE_MS);
  if (runStateSurvived() && runState.towInProgress) {
    // A reset hit in the middle of the tow: finish it safely and don't tow again this run.
    Serial.println(F("A reset interrupted the tow: lifting it, tow point skipped this run"));
    for (uint8_t i = 0; i < 3; i++) {
      digitalWrite(LED, HIGH);
      delay(150);
      digitalWrite(LED, LOW);
      delay(150);
    }
    if (runState.towIsDown) {
      towUp();
      delay(500);
    }
    markerRecorded[(uint8_t)TOW_MARKER] = true;
  } else if (TOW_LIFT_AT_BOOT) {
    towUp();
    delay(500);
  }
  // Fresh tow state for this run.
  memset(&runState, 0, sizeof(runState));
  runState.magic = RUN_MAGIC;
  saveRunState();
  Serial.println(F("Starting platform (radar homes to its start angle)"));
  d9.begin();
  delay(500);
  Serial.println(F("Starting in 5 s"));
  unsigned long start = millis();
  while (millis() - start < START_DELAY_MS) {
    digitalWrite(LED, (millis() / 500) % 2);
  }
  startDriving();
}

void loop() {
  unsigned long now = millis();

  if (phase == DRIVING) {
    digitalWrite(LED, HIGH);
    followLine();
    recordDriving();
    d9.getServoScanner().update();
    if (laserHit()) {
      threatFound();
      return;
    }
    int8_t marker = newMarker();
    if (marker == TOW_MARKER) {
      runTow(marker);
    } else if (marker != NO_MARKER) {
      startMarkerScan(marker);
    }
  } else if (phase == MARKER_SCAN) {
    digitalWrite(LED, (now / 250) % 2);
    sweepRadar();
    if (laserHit()) {
      threatFound();
    } else if (sweepsDone >= MARKER_SWEEPS) {
      logTime();
      Serial.println(F("No threat here, driving on"));
      startDriving();
    }
  } else {
    digitalWrite(LED, LOW);
    if (now - phaseStart >= THREAT_HOLD_MS) {
      startDriving();
    }
  }
}

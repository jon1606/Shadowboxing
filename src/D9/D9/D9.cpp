#include "D9.h"

// The radar sweeps 50-245 deg on a 270-degree servo (500-2500 us across its full travel);
// the ends were found with mainRadarCalibrate.cpp so the sweep is even on both sides.
#define RADAR_MIN_ANGLE 50
#define RADAR_MAX_ANGLE 245
#define RADAR_STEP_MS 17
#define RADAR_SERVO_RANGE 270
#define RADAR_MIN_PULSE_US 500
#define RADAR_MAX_PULSE_US 2500

// A tow pulls backwards for TOW_REVERSE_MS, with a pause on each side for the hook to settle.
#define TOW_REVERSE_MS 7000
#define TOW_SETTLE_MS 500

D9Platform::D9Platform(
    GenericPlatform& basePlatform,
    uint8_t ldrPin,
    uint16_t laserThreshold,
    uint8_t radarPin,
    uint8_t towPin
)
    : genericPlatform(basePlatform),
      enemyDetector(ldrPin, laserThreshold),
      servoScanner(
          radarPin,
          RADAR_MIN_ANGLE, RADAR_MAX_ANGLE, RADAR_STEP_MS, RADAR_MIN_ANGLE,
          RADAR_SERVO_RANGE, RADAR_MIN_PULSE_US, RADAR_MAX_PULSE_US
      ),
      towHook(towPin),
      trackPilot(basePlatform),
      radarPasses(0),
      radarLastEnd(RADAR_MIN_ANGLE) {}

void D9Platform::begin() {
    genericPlatform.begin();
    enemyDetector.begin();
    trackPilot.begin();
    // The hook goes before the radar, so two servos never start moving at the same moment
    // (the current spike can reset the Arduino).
    towHook.begin();
    restartRadar();
}

// Pushes off from standstill; call followLine() from then on.
void D9Platform::startDriving() {
    trackPilot.start();
}

void D9Platform::followLine() {
    trackPilot.followLine();
}

void D9Platform::stopDriving() {
    trackPilot.stop();
}

// Restarts the radar sweep from its start angle and counts passes from zero.
void D9Platform::restartRadar() {
    servoScanner.begin();
    radarPasses = 0;
    radarLastEnd = RADAR_MIN_ANGLE;
}

// Steps the radar sweep and counts a pass each time it reaches the opposite end.
void D9Platform::sweepRadar() {
    servoScanner.update();
    uint8_t angle = servoScanner.getAngle();
    uint8_t end = radarLastEnd;
    if (angle >= RADAR_MAX_ANGLE) {
        end = RADAR_MAX_ANGLE;
    } else if (angle <= RADAR_MIN_ANGLE) {
        end = RADAR_MIN_ANGLE;
    }
    if (end != radarLastEnd) {
        radarLastEnd = end;
        radarPasses++;
    }
}

// Stops the sweep; the radar keeps pointing where it is.
void D9Platform::holdRadar() {
    servoScanner.stop();
}

uint8_t D9Platform::getRadarAngle() const {
    return servoScanner.getAngle();
}

uint8_t D9Platform::getRadarPasses() const {
    return radarPasses;
}

// True once per laser hit on the LDR.
bool D9Platform::threatDetected() {
    return enemyDetector.newHit();
}

// Tows: lowers the hook, retraces the path backwards for TOW_REVERSE_MS and lifts the hook.
void D9Platform::tow() {
    trackPilot.stop();
    holdRadar();
    towHook.lower();
    delay(TOW_SETTLE_MS);
    trackPilot.reverseAlongPath(TOW_REVERSE_MS);
    delay(TOW_SETTLE_MS);
    towHook.lift();
}

// True if a reset cut the last tow short; begin() has lifted the hook again.
bool D9Platform::towWasInterrupted() const {
    return towHook.wasInterrupted();
}

GenericPlatform& D9Platform::getGenericPlatform() {
    return genericPlatform;
}

EnemyDetector& D9Platform::getEnemyDetector() {
    return enemyDetector;
}

ServoScanner& D9Platform::getServoScanner() {
    return servoScanner;
}

TowHook& D9Platform::getTowHook() {
    return towHook;
}

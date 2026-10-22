#include "D9.h"

// How long findEnemy() scans at one stop before giving up and letting the route continue.
#define SCAN_TIMEOUT_MS 20000UL

// The radar sweeps 0-180 deg on a 270-degree servo (500-2500 us across its full travel).
#define RADAR_MIN_ANGLE 0
#define RADAR_MAX_ANGLE 180
#define RADAR_STEP_MS 17
#define RADAR_SERVO_RANGE 270
#define RADAR_MIN_PULSE_US 500
#define RADAR_MAX_PULSE_US 2500

D9Platform::D9Platform(
    GenericPlatform& basePlatform,
    uint8_t ldrPin,
    uint16_t laserThreshold,
    uint8_t servoPin
)
    : genericPlatform(basePlatform),
      enemyDetector(ldrPin, laserThreshold),
      servoScanner(
          servoPin,
          RADAR_MIN_ANGLE, RADAR_MAX_ANGLE, RADAR_STEP_MS, RADAR_MIN_ANGLE,
          RADAR_SERVO_RANGE, RADAR_MIN_PULSE_US, RADAR_MAX_PULSE_US
      ) {}

void D9Platform::begin() {
    genericPlatform.begin();
    enemyDetector.begin();
    servoScanner.begin();
}

// Sweeps the radar until the LDR sees the laser and reports one THREAT to the tank.
// Returns false if nothing was found within SCAN_TIMEOUT_MS; ends the mission on STOP.
bool D9Platform::findEnemy() {
    // begin() restarts the sweep from 0 deg, so every stop gets a fresh scan.
    servoScanner.begin();
    unsigned long start = millis();
    while (millis() - start < SCAN_TIMEOUT_MS) {
        genericPlatform.listen();
        if (genericPlatform.needToStop()) {
            servoScanner.stop();
            genericPlatform.endMission();
        }
        servoScanner.update();
        if (enemyDetector.detected()) {
            servoScanner.stop();
            genericPlatform.sendMessage("TNK", "THREAT", genericPlatform.totalDistance(), servoScanner.getAngle());
            return true;
        }
    }
    servoScanner.stop();
    return false;
}

void D9Platform::tow() {
    servoScanner.moveToAngle(90);
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

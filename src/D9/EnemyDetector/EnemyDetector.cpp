#include "EnemyDetector.h"

// A hit counts once the laser has been seen for this many readings in a row (filters flickers).
#define HIT_CONFIRM_READS 3

// After a hit, the laser has to be gone this long before the next hit can count.
#define REARM_MS 500

EnemyDetector::EnemyDetector(uint8_t pin, uint16_t detectionThreshold)
    : ldrPin(pin),
      threshold(detectionThreshold),
      lastReading(0),
      hitReads(0),
      armed(true),
      laserGoneSince(0) {}

void EnemyDetector::begin() {
    pinMode(ldrPin, INPUT);
    hitReads = 0;
    armed = true;
}

// True while the laser is on the LDR: more light gives a lower reading.
bool EnemyDetector::detected() {
    lastReading = analogRead(ldrPin);
    return lastReading < threshold;
}

// True once per laser hit: after HIT_CONFIRM_READS readings in a row with the laser on. The same
// laser can't count twice: it has to be gone for REARM_MS first.
bool EnemyDetector::newHit() {
    bool laser = detected();
    if (!armed) {
        if (laser) {
            laserGoneSince = millis();
        } else if (millis() - laserGoneSince >= REARM_MS) {
            armed = true;
        }
        return false;
    }

    hitReads = laser ? hitReads + 1 : 0;
    if (hitReads < HIT_CONFIRM_READS) {
        return false;
    }
    hitReads = 0;
    armed = false;
    laserGoneSince = millis();
    return true;
}

uint16_t EnemyDetector::readLight() {
    lastReading = analogRead(ldrPin);
    return lastReading;
}

uint16_t EnemyDetector::getLastReading() const {
    return lastReading;
}

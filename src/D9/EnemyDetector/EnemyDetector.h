#ifndef ENEMY_DETECTOR_H
#define ENEMY_DETECTOR_H

#include <Arduino.h>

class EnemyDetector {
private:
    uint8_t ldrPin;
    uint16_t threshold;
    uint16_t lastReading;

    // State of newHit(): readings in a row with the laser on, whether a new hit can count yet,
    // and since when the laser has been gone.
    uint8_t hitReads;
    bool armed;
    unsigned long laserGoneSince;

public:
    EnemyDetector(uint8_t pin, uint16_t detectionThreshold);

    void begin();
    bool detected();
    bool newHit();
    uint16_t readLight();
    uint16_t getLastReading() const;
};

#endif

#ifndef D9_PLATFORM_H
#define D9_PLATFORM_H

#include <Arduino.h>
#include "../../Generic/GenericPlatform/GenericPlatform.h"
#include "../EnemyDetector/EnemyDetector.h"
#include "../../Generic/ServoScanner/ServoScanner.h"

class D9Platform {
private:
    GenericPlatform& genericPlatform;
    EnemyDetector enemyDetector;
    ServoScanner servoScanner;

public:
    D9Platform(
        GenericPlatform& basePlatform,
        uint8_t ldrPin,
        uint16_t laserThreshold,
        uint8_t servoPin
    );

    void begin();
    bool findEnemy();
    void tow();
    GenericPlatform& getGenericPlatform();
    EnemyDetector& getEnemyDetector();
    ServoScanner& getServoScanner();
};

#endif

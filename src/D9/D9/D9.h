#ifndef D9_PLATFORM_H
#define D9_PLATFORM_H

#include <Arduino.h>
#include "../../Generic/GenericPlatform/GenericPlatform.h"
#include "../../Generic/ServoScanner/ServoScanner.h"
#include "../EnemyDetector/EnemyDetector.h"
#include "../TowHook/TowHook.h"
#include "../TrackPilot/TrackPilot.h"

class D9Platform {
private:
    GenericPlatform& genericPlatform;
    EnemyDetector enemyDetector;
    ServoScanner servoScanner;
    TowHook towHook;
    TrackPilot trackPilot;

    // Radar passes since the sweep restarted, and the end of the sweep it touched last.
    uint8_t radarPasses;
    uint8_t radarLastEnd;

public:
    D9Platform(
        GenericPlatform& basePlatform,
        uint8_t ldrPin,
        uint16_t laserThreshold,
        uint8_t radarPin,
        uint8_t towPin = NO_PIN     // NO_PIN: no tow hook fitted
    );

    void begin();

    void startDriving();
    void followLine();
    void stopDriving();

    void restartRadar();
    void sweepRadar();
    void holdRadar();
    uint8_t getRadarAngle() const;
    uint8_t getRadarPasses() const;
    bool threatDetected();

    void tow();
    bool towWasInterrupted() const;

    GenericPlatform& getGenericPlatform();
    EnemyDetector& getEnemyDetector();
    ServoScanner& getServoScanner();
    TowHook& getTowHook();
};

#endif

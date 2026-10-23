#ifndef TRACK_PILOT_H
#define TRACK_PILOT_H

#include <Arduino.h>
#include "../../Generic/GenericPlatform/GenericPlatform.h"

// The D9's line following: gentle-then-hard steering, a kick when starting from standstill, and a
// recording of the last 7 s of driving so the D9 can retrace its path backwards.
// There is one recording per program (see TrackPilot.cpp), so use one TrackPilot.
class TrackPilot {
private:
    GenericPlatform& genericPlatform;

    // 1 steering right, -1 steering left, 0 straight; and when the current correction started.
    int8_t steering;
    unsigned long correctionStart;

    // Track speeds set last, for the recording.
    uint8_t leftPwm;
    uint8_t rightPwm;

    void drive(uint8_t left, uint8_t right);
    void recordPath();

public:
    explicit TrackPilot(GenericPlatform& basePlatform);

    void begin();
    void start();
    void followLine();
    void stop();
    void reverseAlongPath(unsigned long durationMs);
};

#endif

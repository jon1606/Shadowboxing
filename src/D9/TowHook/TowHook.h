#ifndef TOW_HOOK_H
#define TOW_HOOK_H

#include <Arduino.h>
#include <Servo.h>

// Tow hook on an FS90R continuous servo. A continuous servo has no position, so the hook is moved
// by turning it for a calibrated time; it goes limp in between.
class TowHook {
private:
    Servo servo;
    uint8_t servoPin;
    bool interrupted;

    void turn(int speedUs, unsigned long durationMs);
    void saveDown(bool down);

public:
    explicit TowHook(uint8_t pin);

    void begin();
    void lower();
    void lift();
    bool wasInterrupted() const;
};

#endif

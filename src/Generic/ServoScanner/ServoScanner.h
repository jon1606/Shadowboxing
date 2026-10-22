#ifndef SERVO_SCANNER_H
#define SERVO_SCANNER_H

#include <Arduino.h>
#include <Servo.h>

class ServoScanner {
private:
    Servo servo;

    uint8_t servoPin;
    uint8_t angle;
    uint8_t minAngle;
    uint8_t maxAngle;

    int8_t direction;
    uint16_t stepIntervalMs;
    unsigned long lastStepTime;

    bool running;
    bool threatDetected;
    bool movingToTarget;
    uint8_t targetAngle;

    // The servo's full travel and the pulse widths at its two ends; the defaults match Servo::write().
    uint16_t servoRange;
    uint16_t minPulseUs;
    uint16_t maxPulseUs;

    void writeAngle(uint8_t degrees);

public:
    ServoScanner(
        uint8_t pin,
        uint8_t minimumAngle = 0,
        uint8_t maximumAngle = 90,
        uint16_t stepInterval = 17,
        uint8_t homeAngle = 0,
        uint16_t fullRangeDegrees = 180,
        uint16_t minimumPulseUs = 544,
        uint16_t maximumPulseUs = 2400
    );

    void begin();
    void update();
    uint8_t getAngle() const;
    void moveToAngle(uint8_t targetAngle);
    void stop();
    void returnHome();
    void moveToAngleGradually(uint8_t targetAngle);
    Servo getServo();
};

#endif


#include "ServoScanner.h"

ServoScanner::ServoScanner(
    uint8_t pin,
    uint8_t minimumAngle,
    uint8_t maximumAngle,
    uint16_t stepInterval,
    uint8_t homeAngle,
    uint16_t fullRangeDegrees,
    uint16_t minimumPulseUs,
    uint16_t maximumPulseUs
)
    : servoPin(pin),
      angle(minimumAngle),
      minAngle(minimumAngle),
      maxAngle(maximumAngle),
      direction(1),
      stepIntervalMs(stepInterval),
      lastStepTime(0),
      running(true),
      threatDetected(false),
      targetAngle(homeAngle),
      movingToTarget(false),
      servoRange(fullRangeDegrees),
      minPulseUs(minimumPulseUs),
      maxPulseUs(maximumPulseUs) {}

// Sends a real angle to the servo, using its own range and pulse widths.
void ServoScanner::writeAngle(uint8_t degrees) {
    servo.writeMicroseconds(map(degrees, 0, servoRange, minPulseUs, maxPulseUs));
}

void ServoScanner::begin() {
    servo.attach(servoPin, minPulseUs, maxPulseUs);
    angle = minAngle;
    writeAngle(angle);

    lastStepTime = millis();
    running = true;
    threatDetected = false;
    movingToTarget = false;
}

void ServoScanner::update() {
    if (!running || threatDetected) return;

    unsigned long now = millis();

    if (now - lastStepTime < stepIntervalMs) return;

    lastStepTime = now;

    // Gradual movement toward a requested angle
    if (movingToTarget) {
        if (angle < targetAngle) {
            ++angle;
        } else if (angle > targetAngle) {
            --angle;
        }

        writeAngle(angle);

        if (angle == targetAngle) {
            movingToTarget = false;
        }

        return;
    }

    // Normal back-and-forth scanning
    if (direction > 0) {
        if (angle >= maxAngle) {
            direction = -1;
            angle = maxAngle - 1;
        } else {
            ++angle;
        }
    } else {
        if (angle <= minAngle) {
            direction = 1;
            angle = minAngle + 1;
        } else {
            --angle;
        }
    }

    writeAngle(angle);
}

uint8_t ServoScanner::getAngle() const {
    return angle;
}

// Immediate movement
void ServoScanner::moveToAngle(uint8_t target) {
    if (target < minAngle || target > maxAngle) return;

    targetAngle = target;
    angle = target;
    movingToTarget = false;
    Serial.print("Moving to angle: ");
    Serial.println(angle);
    writeAngle(angle);
}

// Gradual movement
void ServoScanner::moveToAngleGradually(uint8_t target) {
    if (target < minAngle || target > maxAngle) return;

    targetAngle = target;
    movingToTarget = (angle != target);
    lastStepTime = millis();
}

// Gradually return to home (minimum angle)
void ServoScanner::returnHome() {
    moveToAngleGradually(minAngle);
}

void ServoScanner::stop() {
    running = false;
    movingToTarget = false;
}

Servo ServoScanner::getServo() {
    return servo;
}

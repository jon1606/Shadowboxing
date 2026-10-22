
#include <Arduino.h>
#include "Motor.h"

Motor::Motor(uint8_t pin1, uint8_t pin2, uint8_t enable)
  : in1(pin1), in2(pin2), enablePin(enable),
    direction(0), deploying(false), deploymentStartTime(0) {}

void Motor::begin() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  stop();
}

// Runs the motor at -255..255: positive forward, negative reverse, 0 stops.
void Motor::setSpeed(int16_t speed) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) {
    stop();
    return;
  }

  int8_t wanted = speed > 0 ? 1 : -1;
  if (wanted != direction) {
    // Cut power while the direction flips.
    analogWrite(enablePin, 0);
    direction = wanted;
  }
  // Always rewrite the direction pins: when they're shared, the other track may have changed them.
  if (wanted > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  analogWrite(enablePin, abs(speed));
}

// Lets the motor coast. Only the enable pin is touched: on robots where both tracks share
// in1/in2, clearing them here would also stop the other track.
void Motor::stop() {
  analogWrite(enablePin, 0);
}

// True when the two motors are wired to the same direction pins, so they can't turn opposite ways.
bool Motor::sharesDirectionPins(const Motor& other) const {
  return in1 == other.in1 && in2 == other.in2;
}

bool Motor::openBridge() {

  if (!deploying) {
    deploymentStartTime = millis();
    setSpeed(200);
    deploying = true;
  }

  if (deploying && (millis() - deploymentStartTime >= 10000)) {
    stop();
    deploying = false;
    return true; // Return true to indicate that the deployment is complete
  }
  return false; // Return false to indicate that the deployment is not yet complete
}

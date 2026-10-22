
#include "Sonic.h"

UltrasonicSensor::UltrasonicSensor(
  uint8_t trigger,
  uint8_t echo
)
  : trigPin(trigger),
    echoPin(echo),
    lastDistance(-1.0) {}

void UltrasonicSensor::begin() {
  if (trigPin == NO_PIN) {
    return;
  }
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, LOW);
}

float UltrasonicSensor::readDistanceCm() {
  // Without a sensor, report "nothing in range" at once.
  if (trigPin == NO_PIN) {
    lastDistance = NO_ECHO_CM;
    return lastDistance;
  }

  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10-microsecond trigger pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo pulse; timeout after 30 ms
  uint16_t duration = pulseIn(echoPin, HIGH, 30000UL);

  // No echo received
  if (duration == 0) {
    lastDistance = NO_ECHO_CM;
    return lastDistance;
  }

  // Calculate distance in centimeters
  lastDistance = duration * 0.034f / 2.0f;

  return lastDistance;
}

float UltrasonicSensor::getLastDistance() {
  return lastDistance;
}

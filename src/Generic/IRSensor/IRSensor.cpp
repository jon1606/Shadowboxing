
#include <Arduino.h>
#include "IRSensor.h"

IRSensor::IRSensor(uint8_t sensorPin, uint16_t thresholdValue) : pin(sensorPin), threshold(thresholdValue) {}

void IRSensor::begin() {
  pinMode(pin, INPUT);
}

bool IRSensor::detectsLine() {
  return analogRead(pin) > threshold;
}

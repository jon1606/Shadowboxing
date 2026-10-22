
#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

// Pass as the trigger and echo pins when a platform has no ultrasonic sensor.
#define NO_PIN 255

// Distance readDistanceCm() returns when no echo comes back (or there is no sensor).
#define NO_ECHO_CM 10000.0f

class UltrasonicSensor {
private:
  uint8_t trigPin;
  uint8_t echoPin;

  float lastDistance;

public:
  UltrasonicSensor(uint8_t trigger, uint8_t echo);

  void begin();

  float readDistanceCm();

  float getLastDistance();
};

#endif

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
  uint8_t in1;
  uint8_t in2;
  uint8_t enablePin;

  // Direction this motor last asked for: 1 forward, -1 reverse, 0 not set yet.
  int8_t direction;

  // State of an openBridge() run.
  bool deploying;
  unsigned long deploymentStartTime;

public:
  Motor(uint8_t pin1, uint8_t pin2, uint8_t enable);

  void begin();
  void setSpeed(int16_t speed);
  bool openBridge();
  void stop();
  bool sharesDirectionPins(const Motor& other) const;
};

#endif


#include "TankDrive.h"

TankDrive::TankDrive(const Motor& left, const Motor& right)
  : leftMotor(left), rightMotor(right) {}

void TankDrive::begin() {
  leftMotor.begin();
  rightMotor.begin();
}

void TankDrive::drive(int leftSpeed, int rightSpeed) {
  // Tracks on shared direction pins can't run opposite ways (the last write would reverse both),
  // so keep the faster track's direction and stop the slower one.
  bool opposite = (leftSpeed > 0 && rightSpeed < 0) || (leftSpeed < 0 && rightSpeed > 0);
  if (opposite && leftMotor.sharesDirectionPins(rightMotor)) {
    if (abs(leftSpeed) >= abs(rightSpeed)) {
      rightSpeed = 0;
    } else {
      leftSpeed = 0;
    }
  }
  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
}

void TankDrive::stop() {
  leftMotor.stop();
  rightMotor.stop();
}

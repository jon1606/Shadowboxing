
#ifndef GENERIC_PLATFORM_H
#define GENERIC_PLATFORM_H

#include <Arduino.h>
#include "../TankDrive/TankDrive.h"
#include "../IRSensor/IRSensor.h"
#include "../LineFollower/LineFollower.h"
#include "../Comm/Comm.h"
#include "../Sonic/Sonic.h"
#include "../IRReceiver/IRReceiver.h"

class GenericPlatform {
private:
  TankDrive driveSystem;
  IRSensor leftSensor;
  IRSensor rightSensor;
  LineFollower lineFollower;
  Comm comunication;
  UltrasonicSensor sonic;
  IRReceiver irReceiver;
  float distanceTravelled;
  bool stop;
  bool tracksStopped;

  // Speeds and time of the last driveTracks() command, for the distance estimate.
  int16_t lastLeftPwm;
  int16_t lastRightPwm;
  unsigned long lastDriveTime;

public:
  GenericPlatform(
    const uint8_t leftIn1, const uint8_t leftIn2, const uint8_t leftEnable,
    const uint8_t rightIn1, const uint8_t rightIn2, const uint8_t rightEnable,
    const uint8_t leftIRPin, const uint8_t rightIRPin,
    const uint8_t commTXD, const uint8_t commRXD,
    const uint8_t sonicTrigPin, const uint8_t sonicEchoPin,
    const uint8_t irReceiverPin,
    const uint16_t threshold, const String& name
  );

  void begin();
  float updateDrive();
  int8_t readLine();
  void driveTracks(int leftPwm, int rightPwm);
  void stopDrive();
  String listen();
  bool sendMessage(const String& to, const String& type,
                   int x, int angle, bool waitForAck = true);
  float checkSonicDistance();
  int8_t updateIRReceiver();
  float totalDistance();
  void sendLocationIfDue();
  void driveUntilFoundIR(byte targetIRValue);
  void endMission();
  void driveUntilTheEnd();
  bool needToStop();
  void driveForXseconds(int seconds);
};

#endif

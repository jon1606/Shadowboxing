#include "GenericPlatform.h"

// Default motor speed used before line-following correction is applied.
#define BASE_PWM 200

// Proportional gain that converts line error into a motor-speed correction.
#define KP 140

// Estimated platform speed at the base PWM value, in centimetres per second.
#define V_BASE_CMPS 12.0f

// Tracks stop while the ultrasonic sensor sees something closer than this, in centimetres.
#define OBSTACLE_CM 3

// Minimum interval between periodic location messages, in milliseconds.
#define LOC_INTERVAL 5000UL

// Value IRReceiver::update() returns when no new code was decoded.
#define NO_IR_CODE 100

// IR marker that every platform ignores.
#define IGNORED_IR_MARKER 12

// Tracks whether location reporting is enabled by higher-level application code.
bool locationReportingActive = false;

// Converts a known IR marker value into the corresponding travelled distance.
static int16_t getDistance(uint8_t ir) {
  switch (ir) {
    case 16: return 497;
    case 15: return 0;
    case 14: return 367;
    case 13: return 348;
    case 11: return 98;
    case 4:  return 219;
    case 3:  return 295;
    case 2:  return 102;
    case 1:  return 0;
    default: return -1;
  }
}

// Constructs the platform's motors, sensors, communication link, and state.
GenericPlatform::GenericPlatform(
  uint8_t li1, uint8_t li2, uint8_t le,
  uint8_t ri1, uint8_t ri2, uint8_t re,
  uint8_t lir, uint8_t rir,
  uint8_t tx, uint8_t rx,
  uint8_t trig, uint8_t echo,
  uint8_t ir, uint16_t threshold, const String& name
)
  : driveSystem(
      Motor(li1, li2, le),
      Motor(ri1, ri2, re)
    ),
    leftSensor(lir, threshold),
    rightSensor(rir, threshold),
    lineFollower(leftSensor, rightSensor),
    comunication(tx, rx, name),
    sonic(trig, echo),
    irReceiver(ir),
    distanceTravelled(0.0f),
    stop(false) {}

// Initializes every hardware subsystem used by the platform.
void GenericPlatform::begin() {
  driveSystem.begin();
  lineFollower.begin();
  comunication.begin();
  sonic.begin();
  irReceiver.begin();
}

// Sends the current travelled distance when the reporting interval has elapsed.
void GenericPlatform::sendLocationIfDue() {

  static unsigned long lastLocationTime = 0;

  unsigned long currentTime = millis();
  if (currentTime - lastLocationTime >= LOC_INTERVAL) {
    // Fire and forget: waiting for the ACK here would leave the tracks driving blind.
    sendMessage(
      "HML",
      "POSITION",
      static_cast<int>(totalDistance()),
      0,
      false
    );
    lastLocationTime = currentTime;
  }
}

// Updates line-following control, estimates distance, and drives the motors.
float GenericPlatform::updateDrive() {
  unsigned long currentTime = millis();
  static unsigned long lastLoopTime = millis();
  float dt = (currentTime - lastLoopTime) / 1000.0f;
  lastLoopTime = currentTime;

  int sensors = lineFollower.update();
  if (sensors == 2) {
    return distanceTravelled;
  }
  // Pause while something is too close; driving resumes on a later call once it's clear.
  if (checkSonicDistance() < OBSTACLE_CM) {
    driveSystem.stop();
    return distanceTravelled;
  }

  uint8_t leftPWM;
  uint8_t rightPWM;
  if (sensors == 0) {
    rightPWM = BASE_PWM;
    leftPWM = BASE_PWM;
  } else if (sensors == -1) {
    rightPWM = BASE_PWM;
    leftPWM = 0;
  } else if (sensors == 1) {

    rightPWM = 0;
    leftPWM = BASE_PWM;
  }
  float vL = V_BASE_CMPS * ((float)leftPWM / BASE_PWM);
  float vR = V_BASE_CMPS * ((float)rightPWM / BASE_PWM);
  distanceTravelled += (vL + vR) * 0.5f * dt;

  driveSystem.drive(leftPWM, rightPWM);
  return distanceTravelled;
}

// Reads and returns the next message received by the communication subsystem.
String GenericPlatform::listen() {
  String message = comunication.listen();
  if (message == "STOP") {
    stop = true;
  }
  return message;
}

// Sends a message and increments the message sequence counter. Returns true once it is ACKed
// (always true when waitForAck is false).
bool GenericPlatform::sendMessage(
  const String& to, const String& type,
  int x, int angle, bool waitForAck
) {
  static int counter = 0;
  bool delivered = comunication.sendMessage(to, type, x, angle, counter, waitForAck);
  counter++;
  // A STOP can arrive while sendMessage() waits for its ACK; don't lose it.
  if (comunication.stopReceived()) {
    stop = true;
  }
  return delivered;
}

// Measures and returns the current distance reported by the ultrasonic sensor.
float GenericPlatform::checkSonicDistance() {
  return sonic.readDistanceCm();
}

// Reads an IR marker and updates the travelled distance when it is recognized.
int8_t GenericPlatform::updateIRReceiver() {
  int8_t ir = irReceiver.update();
  // Marker 12 is ignored everywhere: reported as "nothing received" and never moves the distance.
  if (ir == IGNORED_IR_MARKER) {
    return NO_IR_CODE;
  }
  if (getDistance(ir) != -1) {
    // Replace the estimated distance with the calibrated marker distance.
    distanceTravelled = getDistance(ir);
  }

  return ir;
}

// Immediately stops both drive motors.
void GenericPlatform::stopDrive() {
  driveSystem.stop();
}

// Returns the platform's current estimated travelled distance.
float GenericPlatform::totalDistance() {
  return distanceTravelled;
}

// Drives while processing communication and sensors until the target IR marker appears.
void GenericPlatform::driveUntilFoundIR(byte targetIRValue) {
  while (true) {
    // Continue line following while servicing the platform's other subsystems.
    updateDrive();
    listen();
    if (needToStop()) {
      endMission();
      break;
    }
    sendLocationIfDue();
    int irValue = updateIRReceiver();
    if (irValue == targetIRValue) {
      // Stop as soon as the requested marker is detected.
      stopDrive();
      break;
    }
  }
}

void GenericPlatform::endMission() {
  // Stop the drive motors.
  stopDrive();

  // Send a message to the controller indicating that the mission has ended.
  sendMessage("HML", "FIN", 0, 0);

  // Remain available for incoming messages while periodically reporting location.
  while (true) {
    sendLocationIfDue();
  }
}

void GenericPlatform::driveUntilTheEnd() {
  bool running = true;
  while (running) {
    // Continue line following while servicing the platform's other subsystems.
    updateDrive();
    listen();
    if (needToStop()) {
      endMission();
    }
    sendLocationIfDue();
    if (checkSonicDistance() < 10.0f) {
      // Stop as soon as an obstacle is detected within 10 cm.
      stopDrive();
      running = false;
    }
  }
}

bool GenericPlatform::needToStop() {
  return stop;
}


void GenericPlatform::driveForXseconds(int seconds) {
  unsigned long startTime = millis();
  while (millis() - startTime < seconds * 1000) {
    updateDrive();
    listen();
    if (needToStop()) {
      endMission();
      break;
    }
    sendLocationIfDue();
  }
}

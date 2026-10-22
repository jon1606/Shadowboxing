#ifndef D9_TEST_H
#define D9_TEST_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "../D9/D9.h"
#include "../../Generic/Motor/Motor.h"
#include "../../Generic/TankDrive/TankDrive.h"

// Serial-menu test bench: runs each D9 subsystem on its own or in combos, logging sensor values.
class D9Test {
private:
    // How a test or a test step ended.
    enum Result : uint8_t {
        DONE,
        USER_STOP,
        TIMED_OUT,
        LORA_STOP,
        NOT_RUN
    };

    // Subsystems a line-following run should include.
    enum DriveFlag : uint8_t {
        DRIVE_MOTORS = 1,
        DRIVE_OBSTACLE = 2,
        DRIVE_MARKER = 4,
        DRIVE_LORA = 8,
        DRIVE_REPORTS = 16
    };

    D9Platform& d9;
    GenericPlatform& genericPlatform;

    // Direct track access for single-track and fixed-PWM tests (same pins as the platform's TankDrive).
    Motor leftMotor;
    Motor rightMotor;
    TankDrive tracks;

    // Second handle on the LoRa pins, used to send with an ACK timeout and to sniff raw traffic.
    SoftwareSerial loraRaw;
    const char* platformName;

    // Wiring, kept for raw sensor reads and the pin check.
    uint8_t leftIn1;
    uint8_t leftIn2;
    uint8_t leftEnable;
    uint8_t rightIn1;
    uint8_t rightIn2;
    uint8_t rightEnable;
    uint8_t leftIRPin;
    uint8_t rightIRPin;
    uint16_t lineThreshold;
    uint8_t loraTxdPin;
    uint8_t loraRxdPin;
    uint8_t sonicTrigPin;
    uint8_t sonicEchoPin;
    uint8_t irReceiverPin;
    uint8_t ldrPin;
    uint16_t laserThreshold;
    uint8_t servoPin;

    // Settings that can be changed from the options menu.
    uint16_t logIntervalMs;
    uint16_t motionTimeoutS;
    uint8_t basePwm;
    uint8_t correctionPwm;
    uint8_t obstacleCm;

    // State of the running test.
    unsigned long testStart;
    unsigned long lastLog;
    float estimatedDistance;
    bool servoReady;
    uint16_t messageCounter;
    uint8_t lastIrCode;
    unsigned long lastIrTime;

    // Menu and serial input.
    void printMenu();
    void runCommand(int command);
    void printHeader(const __FlashStringHelper* title);
    void startTest();
    void printResult(Result result);
    bool stopRequested();
    bool waitOrStop(unsigned long ms);
    void drainInput();
    uint8_t readLine(char* buffer, uint8_t size);
    long askNumber(const __FlashStringHelper* question, long defaultValue, long minValue, long maxValue);
    bool askYesNo(const __FlashStringHelper* question, bool defaultYes);

    // Logging.
    bool logDue(bool changed = false);
    void logBegin(const __FlashStringHelper* tag);
    void logField(const __FlashStringHelper* key, long value);
    void logLineSensor(const __FlashStringHelper* key, uint16_t value);
    void logDistance(float distance);
    void logPwm(uint8_t leftPwm, uint8_t rightPwm);
    void printLineState(int8_t state);
    void printRange(const __FlashStringHelper* name, uint16_t low, uint16_t high);
    void printPin(uint8_t pin);
    const __FlashStringHelper* pinRole(uint8_t index);
    void fillPins(uint8_t* pins);
    void printPinMap(bool warningsOnly);

    // Building blocks shared by the tests.
    void stopTracks();
    bool motionTimedOut();
    bool ensureServo();
    int8_t lineState(uint16_t& leftValue, uint16_t& rightValue);
    bool linePwm(int8_t state, uint8_t& leftPwm, uint8_t& rightPwm);
    uint8_t readMarker();
    void printMessage(Print& out, const __FlashStringHelper* to, const __FlashStringHelper* type, int x, int angle);
    long sendWithAck(const __FlashStringHelper* to, const __FlashStringHelper* type, int x, int angle, uint16_t timeoutMs);
    Result followLine(uint8_t flags, uint8_t targetMarker);
    Result scanForThreat(bool report);

    // Single-system tests.
    void testMotor(bool left);
    void testTracks();
    void testLineSensors();
    void testSonic();
    void testIRReceiver();
    void testLoraListen();
    void testLoraSniff();
    void testLoraSend();
    void testLdr();
    void testServoSweep();
    void testServoAngle();
    void testTow();

    // Combos and tools.
    void comboThreat();
    void comboLine(const __FlashStringHelper* title, uint8_t flags);
    void comboMission();
    void sensorDashboard();
    void editOptions();

public:
    D9Test(
        D9Platform& platform,
        uint8_t leftIn1, uint8_t leftIn2, uint8_t leftEnable,
        uint8_t rightIn1, uint8_t rightIn2, uint8_t rightEnable,
        uint8_t leftIRPin, uint8_t rightIRPin, uint16_t lineThreshold,
        uint8_t loraTxdPin, uint8_t loraRxdPin, const char* name,
        uint8_t sonicTrigPin, uint8_t sonicEchoPin,
        uint8_t irReceiverPin,
        uint8_t ldrPin, uint16_t laserThreshold,
        uint8_t servoPin
    );

    void begin();
    void update();
};

#endif

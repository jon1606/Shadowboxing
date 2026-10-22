#include "D9Test.h"
// The IR decoder is compiled in IRReceiver.cpp; this file only reads its results, so include declarations only.
#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE
#include <IRremote.hpp>

// Value IRReceiver::update() returns when no new code was decoded.
#define NO_IR_CODE 100

// Estimated platform speed at BASE_PWM, copied from GenericPlatform.cpp.
#define V_BASE_CMPS 12.0f

// Interval between POSITION reports, copied from GenericPlatform.cpp.
#define LOC_INTERVAL 5000UL

// How long a report waits for its ACK before it counts as lost.
#define REPORT_ACK_TIMEOUT 1500

// The same IR code arriving again within this window is one press and is logged once.
#define IR_REPEAT_WINDOW 500

// Mission rehearsal step codes; anything below SCAN_STEP is an IR marker to drive to.
#define SCAN_STEP 200
#define TOW_STEP 201

// Number of entries fillPins() writes; the servo is the last one.
#define PIN_COUNT 15

D9Test::D9Test(
    D9Platform& platform,
    uint8_t leftIn1, uint8_t leftIn2, uint8_t leftEnable,
    uint8_t rightIn1, uint8_t rightIn2, uint8_t rightEnable,
    uint8_t leftIRPin, uint8_t rightIRPin, uint16_t lineThreshold,
    uint8_t loraTxdPin, uint8_t loraRxdPin, const char* name,
    uint8_t sonicTrigPin, uint8_t sonicEchoPin,
    uint8_t irReceiverPin,
    uint8_t ldrPin, uint16_t laserThreshold,
    uint8_t servoPin
)
    : d9(platform),
      genericPlatform(platform.getGenericPlatform()),
      leftMotor(leftIn1, leftIn2, leftEnable),
      rightMotor(rightIn1, rightIn2, rightEnable),
      tracks(leftMotor, rightMotor),
      // Same argument order as Comm: the module's TXD feeds the Arduino's receive pin.
      loraRaw(loraTxdPin, loraRxdPin),
      platformName(name),
      leftIn1(leftIn1),
      leftIn2(leftIn2),
      leftEnable(leftEnable),
      rightIn1(rightIn1),
      rightIn2(rightIn2),
      rightEnable(rightEnable),
      leftIRPin(leftIRPin),
      rightIRPin(rightIRPin),
      lineThreshold(lineThreshold),
      loraTxdPin(loraTxdPin),
      loraRxdPin(loraRxdPin),
      sonicTrigPin(sonicTrigPin),
      sonicEchoPin(sonicEchoPin),
      irReceiverPin(irReceiverPin),
      ldrPin(ldrPin),
      laserThreshold(laserThreshold),
      servoPin(servoPin),
      logIntervalMs(250),
      motionTimeoutS(30),
      basePwm(200),
      correctionPwm(0),
      obstacleCm(10),
      testStart(0),
      lastLog(0),
      estimatedDistance(0.0f),
      servoReady(false),
      messageCounter(0),
      lastIrCode(NO_IR_CODE),
      lastIrTime(0) {}

// Brings up every subsystem except the servo, then prints the pin check and the menu.
void D9Test::begin() {
    // Start the raw LoRa handle first so the platform's Comm ends up as the active receiver.
    loraRaw.begin(9600);
    tracks.begin();
    genericPlatform.begin();
    d9.getEnemyDetector().begin();
    // The servo is attached on first use instead, so it doesn't jump at boot or grab a shared pin early.

    Serial.print(F("\nD9 test bench, LoRa name "));
    Serial.println(platformName);
    printPinMap(true);
    printMenu();
    Serial.print(F("\n> "));
}

// Waits for a menu command and runs it.
void D9Test::update() {
    if (!Serial.available()) {
        return;
    }
    char buffer[6];
    if (readLine(buffer, sizeof(buffer)) > 0) {
        runCommand(atoi(buffer));
        // Whatever the test did, never leave the tracks running.
        stopTracks();
        drainInput();
    }
    Serial.print(F("\n> "));
}

// Prints the list of tests.
void D9Test::printMenu() {
    Serial.println(F(
        "\n== D9 TEST BENCH == number + Enter runs a test, Enter stops it\n"
        "Systems:  1 left track  2 right track  3 both tracks  4 line sensors\n"
        "  5 ultrasonic  6 IR markers  7 LoRa listen  8 LoRa sniff  9 LoRa ping\n"
        " 10 threat LDR  11 scanner sweep  12 servo angle  13 tow\n"
        "Combos:  20 threat detection  21 line follow  22 line + obstacle\n"
        " 23 drive to marker  24 mission rehearsal\n"
        "Tools:  30 dashboard  31 pin check  32 options  0 menu"));
}

// Runs the test with the given menu number.
void D9Test::runCommand(int command) {
    switch (command) {
        case 0: printMenu(); break;
        case 1: testMotor(true); break;
        case 2: testMotor(false); break;
        case 3: testTracks(); break;
        case 4: testLineSensors(); break;
        case 5: testSonic(); break;
        case 6: testIRReceiver(); break;
        case 7: testLoraListen(); break;
        case 8: testLoraSniff(); break;
        case 9: testLoraSend(); break;
        case 10: testLdr(); break;
        case 11: testServoSweep(); break;
        case 12: testServoAngle(); break;
        case 13: testTow(); break;
        case 20: comboThreat(); break;
        case 21: comboLine(F("LINE FOLLOW"), 0); break;
        case 22: comboLine(F("LINE FOLLOW + OBSTACLE"), DRIVE_OBSTACLE); break;
        case 23: comboLine(F("DRIVE TO MARKER"), DRIVE_OBSTACLE | DRIVE_MARKER | DRIVE_LORA); break;
        case 24: comboMission(); break;
        case 30: sensorDashboard(); break;
        case 31: printPinMap(false); break;
        case 32: editOptions(); break;
        default: Serial.println(F("Unknown, 0 = menu")); break;
    }
}

// Prints a test title.
void D9Test::printHeader(const __FlashStringHelper* title) {
    Serial.print(F("\n== "));
    Serial.println(title);
}

// Resets the test clock and tells the user how to stop.
void D9Test::startTest() {
    Serial.println(F("Running, Enter stops"));
    testStart = millis();
    lastLog = testStart;
}

// Prints how a test ended and how long it ran.
void D9Test::printResult(Result result) {
    Serial.print(F("RESULT "));
    switch (result) {
        case DONE: Serial.print(F("done")); break;
        case USER_STOP: Serial.print(F("stopped")); break;
        case TIMED_OUT: Serial.print(F("timeout")); break;
        case LORA_STOP: Serial.print(F("LoRa STOP")); break;
        case NOT_RUN: Serial.print(F("not run")); break;
    }
    Serial.print(F(" after "));
    Serial.print(millis() - testStart);
    Serial.println(F(" ms"));
}

// Returns true when anything arrives on the serial monitor, which is how a running test is stopped.
bool D9Test::stopRequested() {
    if (!Serial.available()) {
        return false;
    }
    drainInput();
    return true;
}

// Waits for ms milliseconds; returns true if the user asked to stop meanwhile.
bool D9Test::waitOrStop(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        if (stopRequested()) {
            return true;
        }
    }
    return false;
}

// Discards serial input until the line goes quiet, so a leftover Enter doesn't leak into the next prompt.
void D9Test::drainInput() {
    unsigned long quietSince = millis();
    while (millis() - quietSince < 50) {
        if (Serial.available()) {
            Serial.read();
            quietSince = millis();
        }
    }
}

// Reads one line from the serial monitor, echoing it, until Enter; returns its length.
uint8_t D9Test::readLine(char* buffer, uint8_t size) {
    uint8_t length = 0;
    while (true) {
        if (!Serial.available()) {
            continue;
        }
        char c = Serial.read();
        if (c == '\r' || c == '\n') {
            break;
        }
        if ((c == '\b' || c == 127) && length > 0) {
            length--;
            Serial.print(F("\b \b"));
        } else if (c >= ' ' && length < size - 1) {
            buffer[length++] = c;
            Serial.print(c);
        }
    }
    buffer[length] = '\0';
    Serial.println();
    drainInput();
    return length;
}

// Asks for a number; Enter keeps the default and answers are clamped to [minValue, maxValue].
long D9Test::askNumber(const __FlashStringHelper* question, long defaultValue, long minValue, long maxValue) {
    Serial.print(question);
    Serial.print(F(" ["));
    Serial.print(defaultValue);
    Serial.print(F("]: "));
    char buffer[8];
    if (readLine(buffer, sizeof(buffer)) == 0) {
        return defaultValue;
    }
    long value = atol(buffer);
    return constrain(value, minValue, maxValue);
}

// Asks a yes/no question; Enter keeps the default.
bool D9Test::askYesNo(const __FlashStringHelper* question, bool defaultYes) {
    Serial.print(question);
    Serial.print(defaultYes ? F(" (Y/n): ") : F(" (y/N): "));
    char buffer[4];
    if (readLine(buffer, sizeof(buffer)) == 0) {
        return defaultYes;
    }
    return buffer[0] == 'y' || buffer[0] == 'Y' || buffer[0] == '1';
}

// Returns true once per log interval, or sooner (at most every 50 ms) when something changed.
bool D9Test::logDue(bool changed) {
    unsigned long elapsed = millis() - lastLog;
    if (elapsed < logIntervalMs && !(changed && elapsed >= 50)) {
        return false;
    }
    lastLog = millis();
    return true;
}

// Starts a log line: TAG t=<ms since the test started>.
void D9Test::logBegin(const __FlashStringHelper* tag) {
    Serial.print(tag);
    Serial.print(F(" t="));
    Serial.print(millis() - testStart);
}

// Appends " key=value" to the current log line.
void D9Test::logField(const __FlashStringHelper* key, long value) {
    Serial.print(' ');
    Serial.print(key);
    Serial.print('=');
    Serial.print(value);
}

// Appends a line sensor reading, marked with * when it is above the line threshold.
void D9Test::logLineSensor(const __FlashStringHelper* key, uint16_t value) {
    logField(key, value);
    if (value > lineThreshold) {
        Serial.print('*');
    }
}

// Appends an ultrasonic distance in whole cm, or cm=none when no echo came back.
void D9Test::logDistance(float distance) {
    if (distance < NO_ECHO_CM) {
        logField(F("cm"), (long)distance);
    } else {
        Serial.print(F(" cm=none"));
    }
}

// Appends " pwm=left/right".
void D9Test::logPwm(uint8_t leftPwm, uint8_t rightPwm) {
    Serial.print(F(" pwm="));
    Serial.print(leftPwm);
    Serial.print('/');
    Serial.print(rightPwm);
}

// Appends the line follower decision by name.
void D9Test::printLineState(int8_t state) {
    Serial.print(F(" dec="));
    switch (state) {
        case 0: Serial.print(F("STRAIGHT")); break;
        case 1: Serial.print(F("RIGHT")); break;
        case -1: Serial.print(F("LEFT")); break;
        default: Serial.print(F("LOST")); break;
    }
}

// Prints the lowest and highest reading of a sensor and the value halfway between them.
void D9Test::printRange(const __FlashStringHelper* name, uint16_t low, uint16_t high) {
    Serial.print(name);
    Serial.print(F(" min="));
    Serial.print(low);
    Serial.print(F(" max="));
    Serial.print(high);
    if (high > low && high - low > 100) {
        Serial.print(F(" halfway="));
        Serial.print((low + high) / 2);
    }
    Serial.println();
}

// Prints a pin as D7 or A3, or -- when the part isn't fitted.
void D9Test::printPin(uint8_t pin) {
    if (pin == NO_PIN) {
        Serial.print(F("--"));
    } else if (pin >= A0) {
        Serial.print('A');
        Serial.print(pin - A0);
    } else {
        Serial.print('D');
        Serial.print(pin);
    }
}

// Names the pin at the given index of the pin map.
const __FlashStringHelper* D9Test::pinRole(uint8_t index) {
    switch (index) {
        case 0: return F("left IN1");
        case 1: return F("left IN2");
        case 2: return F("left EN");
        case 3: return F("right IN1");
        case 4: return F("right IN2");
        case 5: return F("right EN");
        case 6: return F("line sensor L");
        case 7: return F("line sensor R");
        case 8: return F("LoRa TXD");
        case 9: return F("LoRa RXD");
        case 10: return F("sonic TRIG");
        case 11: return F("sonic ECHO");
        case 12: return F("IR receiver");
        case 13: return F("LDR");
        default: return F("servo");
    }
}

// Writes every pin the D9 uses, in pinRole() order.
void D9Test::fillPins(uint8_t* pins) {
    const uint8_t wiring[PIN_COUNT] = {
        leftIn1, leftIn2, leftEnable, rightIn1, rightIn2, rightEnable,
        leftIRPin, rightIRPin, loraTxdPin, loraRxdPin,
        sonicTrigPin, sonicEchoPin, irReceiverPin, ldrPin, servoPin
    };
    memcpy(pins, wiring, PIN_COUNT);
}

// Lists the pins the D9 uses (or only the problems) and warns about pins that clash.
void D9Test::printPinMap(bool warningsOnly) {
    uint8_t pins[PIN_COUNT];
    fillPins(pins);
    const uint8_t count = PIN_COUNT;
    uint8_t warnings = 0;

    if (!warningsOnly) {
        printHeader(F("PIN CHECK"));
        for (uint8_t i = 0; i < count; i++) {
            printPin(pins[i]);
            Serial.print(' ');
            Serial.println(pinRole(i));
        }
        Serial.println(F("NOTE IN1/IN2 are shared: one direction for both tracks, so they can't turn opposite ways"));
    }

    for (uint8_t i = 0; i < count; i++) {
        for (uint8_t j = i + 1; j < count; j++) {
            // Indexes 0/3 and 1/4 are IN1/IN2, which the D9 wires to both tracks on purpose.
            if (pins[i] != pins[j] || pins[i] == NO_PIN || (i == 0 && j == 3) || (i == 1 && j == 4)) {
                continue;
            }
            warnings++;
            Serial.print(F("WARNING "));
            printPin(pins[i]);
            Serial.print(F(" is both "));
            Serial.print(pinRole(i));
            Serial.print(F(" and "));
            Serial.println(pinRole(j));
        }
        // IRReceiver::begin() turns on IRremote's LED feedback, which toggles LED_BUILTIN whenever IR arrives.
        if (pins[i] == LED_BUILTIN) {
            warnings++;
            Serial.print(F("WARNING "));
            Serial.print(pinRole(i));
            Serial.println(F(" is on D13, which IR LED feedback toggles"));
        }
    }

    // The Servo library takes PWM away from D9/D10 and IRremote from D3/D11, leaving D5/D6 for the tracks.
    for (uint8_t i = 2; i <= 5; i += 3) {
        if (pins[i] != 5 && pins[i] != 6) {
            warnings++;
            Serial.print(F("WARNING "));
            Serial.print(pinRole(i));
            Serial.println(F(" needs D5 or D6 for PWM"));
        }
    }

    if (warnings == 0) {
        Serial.println(F("Pin check OK"));
    }
}

// Stops both tracks.
void D9Test::stopTracks() {
    tracks.stop();
}

// Returns true (and says so) once the tracks have run for the motion timeout.
bool D9Test::motionTimedOut() {
    if (millis() - testStart < motionTimeoutS * 1000UL) {
        return false;
    }
    Serial.println(F("Motion timeout (32 options)"));
    return true;
}

// Attaches the scanner servo the first time a test needs it. Refuses when its pin is used by
// something else: on the LoRa receive pin, the servo pulses fight the radio and Comm::listen() never returns.
bool D9Test::ensureServo() {
    uint8_t pins[PIN_COUNT];
    fillPins(pins);
    for (uint8_t i = 0; i < PIN_COUNT - 1; i++) {
        if (pins[i] == servoPin) {
            Serial.print(F("Servo not attached: its pin is also "));
            Serial.print(pinRole(i));
            Serial.println(F(". Set SERVO_PIN to the pin the servo is wired to."));
            return false;
        }
    }
    if (!servoReady) {
        d9.getServoScanner().begin();
        servoReady = true;
        Serial.println(F("Servo attached, at 0 deg"));
    }
    return true;
}

// Reads both line sensors and returns the decision LineFollower::update() would make:
// 0 both on the line, 1 left only, -1 right only, 2 neither.
int8_t D9Test::lineState(uint16_t& leftValue, uint16_t& rightValue) {
    leftValue = analogRead(leftIRPin);
    rightValue = analogRead(rightIRPin);
    bool left = leftValue > lineThreshold;
    bool right = rightValue > lineThreshold;
    if (left && right) {
        return 0;
    }
    if (left) {
        return 1;
    }
    if (right) {
        return -1;
    }
    return 2;
}

// Picks track PWMs for a line decision like GenericPlatform::updateDrive();
// returns false when the line is lost, leaving the last PWMs in place.
bool D9Test::linePwm(int8_t state, uint8_t& leftPwm, uint8_t& rightPwm) {
    if (state == 0) {
        leftPwm = basePwm;
        rightPwm = basePwm;
    } else if (state == -1) {
        leftPwm = correctionPwm;
        rightPwm = basePwm;
    } else if (state == 1) {
        leftPwm = basePwm;
        rightPwm = correctionPwm;
    } else {
        return false;
    }
    return true;
}

// Reads the IR receiver through GenericPlatform::updateIRReceiver() and logs each new code.
uint8_t D9Test::readMarker() {
    float distanceBefore = genericPlatform.totalDistance();
    uint8_t code = (uint8_t)genericPlatform.updateIRReceiver();
    if (code == NO_IR_CODE) {
        return code;
    }

    unsigned long now = millis();
    bool repeat = code == lastIrCode && now - lastIrTime < IR_REPEAT_WINDOW;
    lastIrCode = code;
    lastIrTime = now;
    if (repeat) {
        return code;
    }

    logBegin(F("IR"));
    logField(F("code"), code);
    Serial.print(F(" addr=0x"));
    Serial.print(IrReceiver.decodedIRData.address, HEX);
    // updateIRReceiver() moves the platform distance to the calibrated value when the code is a known marker.
    float distanceAfter = genericPlatform.totalDistance();
    logField(F("platformCm"), (long)distanceAfter);
    if (distanceAfter != distanceBefore) {
        estimatedDistance = distanceAfter;
    }
    Serial.println();
    return code;
}

// Writes a message in Comm's wire format: sender|to|type|x|angle|counter.
void D9Test::printMessage(Print& out, const __FlashStringHelper* to, const __FlashStringHelper* type, int x, int angle) {
    out.print(platformName);
    out.print('|');
    out.print(to);
    out.print('|');
    out.print(type);
    out.print('|');
    out.print(x);
    out.print('|');
    out.print(angle);
    out.print('|');
    out.print(messageCounter);
}

// Sends a message like Comm::sendMessage() but gives up on the ACK after timeoutMs.
// Returns the round trip in ms, or -1 when no ACK came (or the user pressed Enter).
long D9Test::sendWithAck(const __FlashStringHelper* to, const __FlashStringHelper* type, int x, int angle, uint16_t timeoutMs) {
    logBegin(F("LORA tx"));
    Serial.print(' ');
    printMessage(Serial, to, type, x, angle);
    Serial.println();
    printMessage(loraRaw, to, type, x, angle);
    loraRaw.println();
    messageCounter++;

    unsigned long sent = millis();
    while (millis() - sent < timeoutMs) {
        // Leave the key press for the caller, which decides how to stop.
        if (Serial.available()) {
            return -1;
        }
        String reply = genericPlatform.listen();
        if (reply.length() == 0) {
            continue;
        }
        long roundTrip = millis() - sent;
        logBegin(F("LORA rx"));
        Serial.print(' ');
        Serial.print(reply);
        if (reply.startsWith("ACK")) {
            logField(F("ms"), roundTrip);
            Serial.println();
            return roundTrip;
        }
        Serial.println();
        if (reply == "STOP") {
            return -1;
        }
    }
    logBegin(F("LORA"));
    Serial.println(F(" no ACK"));
    return -1;
}

// Line-follows the way updateDrive() does, adding obstacle stop, marker stop, LoRa and reports as flags ask.
D9Test::Result D9Test::followLine(uint8_t flags, uint8_t targetMarker) {
    bool motorsOn = flags & DRIVE_MOTORS;
    uint8_t leftPwm = 0;
    uint8_t rightPwm = 0;
    bool blocked = false;
    float distance = NO_ECHO_CM;
    // Loops spent straight, right, left and lost since the last log line.
    uint16_t mix[4] = {0, 0, 0, 0};
    unsigned long start = millis();
    unsigned long lastStep = start;
    unsigned long lastReport = start;
    unsigned long windowStart = start;

    while (true) {
        if (stopRequested()) {
            stopTracks();
            return USER_STOP;
        }
        unsigned long now = millis();
        if (motorsOn && now - start >= motionTimeoutS * 1000UL) {
            stopTracks();
            return TIMED_OUT;
        }

        uint16_t leftValue;
        uint16_t rightValue;
        int8_t state = lineState(leftValue, rightValue);
        bool lost = !linePwm(state, leftPwm, rightPwm);
        mix[state == 0 ? 0 : state == 1 ? 1 : state == -1 ? 2 : 3]++;

        if (flags & DRIVE_OBSTACLE) {
            distance = genericPlatform.checkSonicDistance();
            bool nowBlocked = distance < obstacleCm;
            if (nowBlocked != blocked) {
                blocked = nowBlocked;
                logBegin(blocked ? F("OBSTACLE") : F("CLEAR"));
                logDistance(distance);
                Serial.println();
            }
        }

        bool moving = motorsOn && !blocked && (leftPwm > 0 || rightPwm > 0);
        if (moving) {
            tracks.drive(leftPwm, rightPwm);
        } else {
            stopTracks();
        }
        float seconds = (now - lastStep) / 1000.0f;
        lastStep = now;
        if (moving && basePwm > 0) {
            estimatedDistance += V_BASE_CMPS * ((leftPwm + rightPwm) * 0.5f / basePwm) * seconds;
        }

        if ((flags & DRIVE_MARKER) && readMarker() == targetMarker) {
            stopTracks();
            logBegin(F("MARKER"));
            logField(F("reached"), targetMarker);
            logField(F("est"), (long)estimatedDistance);
            Serial.println();
            return DONE;
        }

        if (flags & DRIVE_LORA) {
            String message = genericPlatform.listen();
            if (message.length() > 0) {
                logBegin(F("LORA rx"));
                Serial.print(' ');
                Serial.println(message);
                if (message == "STOP") {
                    stopTracks();
                    return LORA_STOP;
                }
            }
            if ((flags & DRIVE_REPORTS) && now - lastReport >= LOC_INTERVAL) {
                lastReport = now;
                // Like sendLocationIfDue(), the tracks keep their last command while waiting for the ACK.
                sendWithAck(F("HML"), F("POSITION"), (int)estimatedDistance, 0, REPORT_ACK_TIMEOUT);
            }
        }

        if (logDue()) {
            unsigned long window = millis() - windowStart;
            logBegin(motorsOn ? F("LINE") : F("LINE-DRY"));
            logLineSensor(F("irL"), leftValue);
            logLineSensor(F("irR"), rightValue);
            printLineState(state);
            logPwm(leftPwm, rightPwm);
            if (lost) {
                Serial.print(F(" keep"));
            }
            if (flags & DRIVE_OBSTACLE) {
                logDistance(distance);
            }
            if (blocked) {
                Serial.print(F(" BLOCKED"));
            }
            logField(F("est"), (long)estimatedDistance);
            if (window > 0) {
                logField(F("hz"), (mix[0] + mix[1] + mix[2] + mix[3]) * 1000UL / window);
            }
            Serial.print(F(" mix="));
            for (uint8_t i = 0; i < 4; i++) {
                if (i > 0) {
                    Serial.print('/');
                }
                Serial.print(mix[i]);
                mix[i] = 0;
            }
            Serial.println();
            windowStart = millis();
        }
    }
}

// Sweeps the scanner and watches the LDR like D9Platform::findEnemy(), logging as it goes.
D9Test::Result D9Test::scanForThreat(bool report) {
    ServoScanner& scanner = d9.getServoScanner();
    EnemyDetector& detector = d9.getEnemyDetector();
    if (!ensureServo()) {
        return NOT_RUN;
    }
    // begin() is the only way to restart a ServoScanner after stop(); it also homes it to 0.
    scanner.begin();

    uint16_t light = detector.readLight();
    logBegin(F("SCAN"));
    logField(F("ldr"), light);
    logField(F("thr"), laserThreshold);
    Serial.println();
    if (light < laserThreshold) {
        Serial.println(F("WARNING LDR already under threshold, check 10"));
    }

    uint16_t darkest = 1023;
    uint8_t darkestAngle = 0;
    unsigned long start = millis();
    while (true) {
        if (stopRequested()) {
            scanner.stop();
            return USER_STOP;
        }
        // Same limit as SCAN_TIMEOUT_MS in D9.cpp.
        if (millis() - start >= 20000UL) {
            scanner.stop();
            return TIMED_OUT;
        }
        scanner.update();
        bool detected = detector.detected();
        light = detector.getLastReading();
        uint8_t angle = scanner.getAngle();
        if (light < darkest) {
            darkest = light;
            darkestAngle = angle;
        }

        if (detected) {
            scanner.stop();
            int x = (int)genericPlatform.totalDistance();
            logBegin(F("THREAT"));
            logField(F("angle"), angle);
            logField(F("ldr"), light);
            Serial.print(F(" msg="));
            printMessage(Serial, F("TNK"), F("THREAT"), x, angle);
            Serial.println();
            if (report) {
                sendWithAck(F("TNK"), F("THREAT"), x, angle, 2000);
            }
            return DONE;
        }

        if (logDue()) {
            logBegin(F("SCAN"));
            logField(F("angle"), angle);
            logField(F("ldr"), light);
            // Darkest reading since the last line, so a short laser hit between lines still shows up.
            logField(F("darkest"), darkest);
            Serial.print('@');
            Serial.println(darkestAngle);
            darkest = 1023;
        }
    }
}

// Runs one track alone, at a fixed PWM or ramping up to find the PWM where it starts turning.
void D9Test::testMotor(bool left) {
    printHeader(left ? F("LEFT TRACK") : F("RIGHT TRACK"));
    Motor& motor = left ? leftMotor : rightMotor;
    if (askNumber(F("1 fixed PWM, 2 ramp up"), 1, 1, 2) == 1) {
        uint8_t pwm = askNumber(F("PWM"), basePwm, 0, 255);
        startTest();
        motor.setSpeed(pwm);
        while (!stopRequested() && !motionTimedOut()) {
            if (logDue()) {
                logBegin(F("TRACK"));
                logField(F("pwm"), pwm);
                Serial.println();
            }
        }
    } else {
        Serial.println(F("+5 PWM every 300 ms, press Enter when the track starts turning"));
        startTest();
        for (uint16_t pwm = 0; pwm <= 255; pwm += 5) {
            motor.setSpeed(pwm);
            logBegin(F("RAMP"));
            logField(F("pwm"), pwm);
            Serial.println();
            if (waitOrStop(300)) {
                Serial.print(F("Starts turning at PWM "));
                Serial.println(pwm);
                break;
            }
        }
    }
    stopTracks();
}

// Drives both tracks with PWMs you choose, or plays the PWM pairs updateDrive() uses to steer.
void D9Test::testTracks() {
    printHeader(F("BOTH TRACKS"));
    if (askNumber(F("1 custom PWM, 2 steering pattern"), 2, 1, 2) == 1) {
        uint8_t leftPwm = askNumber(F("Left PWM"), basePwm, 0, 255);
        uint8_t rightPwm = askNumber(F("Right PWM"), basePwm, 0, 255);
        startTest();
        tracks.drive(leftPwm, rightPwm);
        while (!stopRequested() && !motionTimedOut()) {
            if (logDue()) {
                logBegin(F("TRACKS"));
                logPwm(leftPwm, rightPwm);
                Serial.println();
            }
        }
    } else {
        // Straight, steer right, straight, steer left, 2 s each: the pairs updateDrive() sends.
        const uint8_t pattern[4][2] = {
            {basePwm, basePwm},
            {basePwm, correctionPwm},
            {basePwm, basePwm},
            {correctionPwm, basePwm}
        };
        startTest();
        for (uint8_t step = 0; step < 4; step++) {
            tracks.drive(pattern[step][0], pattern[step][1]);
            logBegin(F("TRACKS"));
            logPwm(pattern[step][0], pattern[step][1]);
            Serial.println();
            if (waitOrStop(2000)) {
                break;
            }
        }
    }
    stopTracks();
}

// Logs both line sensors and the decision, then shows the range each sensor saw.
void D9Test::testLineSensors() {
    printHeader(F("LINE SENSORS (* = on line)"));
    uint16_t leftLow = 1023;
    uint16_t leftHigh = 0;
    uint16_t rightLow = 1023;
    uint16_t rightHigh = 0;
    int8_t lastState = 3;
    startTest();
    while (!stopRequested()) {
        uint16_t leftValue;
        uint16_t rightValue;
        int8_t state = lineState(leftValue, rightValue);
        leftLow = min(leftLow, leftValue);
        leftHigh = max(leftHigh, leftValue);
        rightLow = min(rightLow, rightValue);
        rightHigh = max(rightHigh, rightValue);
        if (logDue(state != lastState)) {
            logBegin(F("LINE"));
            logLineSensor(F("irL"), leftValue);
            logLineSensor(F("irR"), rightValue);
            printLineState(state);
            Serial.println();
            lastState = state;
        }
    }
    printRange(F("irL"), leftLow, leftHigh);
    printRange(F("irR"), rightLow, rightHigh);
}

// Logs ultrasonic distance, how long each read blocks, and how often no echo comes back.
void D9Test::testSonic() {
    printHeader(F("ULTRASONIC"));
    uint16_t readings = 0;
    uint16_t misses = 0;
    uint16_t nearest = 10000;
    uint16_t farthest = 0;
    startTest();
    while (!stopRequested()) {
        unsigned long readStart = micros();
        float distance = genericPlatform.checkSonicDistance();
        unsigned long readUs = micros() - readStart;
        readings++;
        if (distance < NO_ECHO_CM) {
            nearest = min(nearest, (uint16_t)distance);
            farthest = max(farthest, (uint16_t)distance);
        } else {
            misses++;
        }
        if (logDue()) {
            logBegin(F("SONIC"));
            logDistance(distance);
            logField(F("readUs"), readUs);
            Serial.println();
        }
        delay(60);
    }
    Serial.print(F("readings="));
    Serial.print(readings);
    Serial.print(F(" noEcho="));
    Serial.println(misses);
    if (readings > misses) {
        printRange(F("cm"), nearest, farthest);
    }
}

// Logs every IR code the receiver decodes, with its address and the platform's marker distance.
void D9Test::testIRReceiver() {
    printHeader(F("IR MARKERS (mission: N 2,4  S 11,13,16)"));
    uint16_t frames = 0;
    lastIrCode = NO_IR_CODE;
    startTest();
    unsigned long lastHeard = millis();
    while (!stopRequested()) {
        if (readMarker() != NO_IR_CODE) {
            frames++;
            lastHeard = millis();
        }
        if (millis() - lastHeard >= 5000) {
            logBegin(F("IR"));
            Serial.println(F(" nothing yet"));
            lastHeard = millis();
        }
    }
    Serial.print(F("frames="));
    Serial.println(frames);
}

// Logs every message the platform's Comm returns (it ACKs messages addressed to this D9 by itself).
void D9Test::testLoraListen() {
    printHeader(F("LORA LISTEN (only messages to this name or ALL; 8 shows all)"));
    uint16_t messages = 0;
    startTest();
    unsigned long lastHeard = millis();
    while (!stopRequested()) {
        String message = genericPlatform.listen();
        if (message.length() > 0) {
            messages++;
            lastHeard = millis();
            logBegin(F("LORA rx"));
            Serial.print(' ');
            Serial.println(message);
        }
        if (millis() - lastHeard >= 5000) {
            logBegin(F("LORA"));
            Serial.println(F(" nothing yet"));
            lastHeard = millis();
        }
    }
    Serial.print(F("messages="));
    Serial.println(messages);
}

// Prints every raw line on the LoRa link, whoever it is addressed to, without ACKing anything.
void D9Test::testLoraSniff() {
    printHeader(F("LORA SNIFF (raw, no ACKs)"));
    loraRaw.listen();
    char line[64];
    uint8_t length = 0;
    uint16_t lines = 0;
    startTest();
    unsigned long lastHeard = millis();
    while (!stopRequested()) {
        while (loraRaw.available()) {
            char c = loraRaw.read();
            lastHeard = millis();
            if (c == '\r') {
                continue;
            }
            if (c != '\n' && length < sizeof(line) - 1) {
                line[length++] = c;
                continue;
            }
            line[length] = '\0';
            length = 0;
            lines++;
            logBegin(F("RAW"));
            Serial.print(' ');
            Serial.println(line);
        }
        if (millis() - lastHeard >= 5000) {
            logBegin(F("RAW"));
            Serial.println(F(" nothing yet"));
            lastHeard = millis();
        }
    }
    // Comm::begin() is the only way to hand the radio back to the platform's receiver.
    genericPlatform.begin();
    Serial.print(F("lines="));
    Serial.println(lines);
}

// Sends PINGs and measures the ACK round trip of each (any node running Comm ACKs them).
void D9Test::testLoraSend() {
    printHeader(F("LORA PING"));
    long target = askNumber(F("To 1 HML, 2 TNK, 3 ALL"), 1, 1, 3);
    const __FlashStringHelper* to = target == 1 ? F("HML") : target == 2 ? F("TNK") : F("ALL");
    uint8_t count = askNumber(F("Pings"), 5, 1, 100);
    uint16_t timeoutMs = askNumber(F("ACK timeout ms"), 2000, 100, 10000);
    uint8_t sent = 0;
    uint8_t acked = 0;
    long slowest = 0;
    long total = 0;
    startTest();
    for (uint8_t i = 0; i < count; i++) {
        long roundTrip = sendWithAck(to, F("PING"), i, 0, timeoutMs);
        sent++;
        if (roundTrip >= 0) {
            acked++;
            total += roundTrip;
            slowest = max(slowest, roundTrip);
        }
        if (waitOrStop(300)) {
            break;
        }
    }
    Serial.print(F("acked "));
    Serial.print(acked);
    Serial.print('/');
    Serial.print(sent);
    if (acked > 0) {
        logField(F("avgMs"), total / acked);
        logField(F("maxMs"), slowest);
    }
    Serial.println();
}

// Logs the threat LDR, flags readings under the laser threshold and shows the range it saw.
void D9Test::testLdr() {
    printHeader(F("THREAT LDR (laser on should read low)"));
    EnemyDetector& detector = d9.getEnemyDetector();
    uint16_t low = 1023;
    uint16_t high = 0;
    bool lastDetected = false;
    startTest();
    while (!stopRequested()) {
        bool detected = detector.detected();
        uint16_t light = detector.getLastReading();
        low = min(low, light);
        high = max(high, light);
        if (logDue(detected != lastDetected)) {
            logBegin(F("LDR"));
            logField(F("ldr"), light);
            logField(F("thr"), laserThreshold);
            if (detected) {
                Serial.print(F(" THREAT"));
            }
            Serial.println();
            lastDetected = detected;
        }
    }
    printRange(F("ldr"), low, high);
}

// Runs the real ServoScanner sweep (0-180 deg, 1 deg every 17 ms) and logs the angle.
void D9Test::testServoSweep() {
    printHeader(F("SCANNER SWEEP"));
    if (!ensureServo()) {
        return;
    }
    ServoScanner& scanner = d9.getServoScanner();
    scanner.begin();
    startTest();
    while (!stopRequested()) {
        scanner.update();
        if (logDue()) {
            logBegin(F("SWEEP"));
            logField(F("angle"), scanner.getAngle());
            Serial.println();
        }
    }
    scanner.stop();
}

// Moves the scanner servo to angles you type, using ServoScanner::moveToAngle().
void D9Test::testServoAngle() {
    printHeader(F("SERVO ANGLE"));
    if (!ensureServo()) {
        return;
    }
    while (true) {
        Serial.print(F("Angle 0-180, Enter to finish: "));
        char buffer[6];
        if (readLine(buffer, sizeof(buffer)) == 0) {
            return;
        }
        long angle = atol(buffer);
        if (angle >= 0 && angle <= 180) {
            d9.getServoScanner().moveToAngle(angle);
        }
    }
}

// Calls d9.tow() (scanner servo to 90 deg) and holds until you release it.
void D9Test::testTow() {
    printHeader(F("TOW"));
    if (!ensureServo()) {
        return;
    }
    startTest();
    d9.tow();
    Serial.println(F("Holding, Enter releases"));
    while (!stopRequested()) {
    }
    d9.getServoScanner().moveToAngle(0);
}

// Threat detection combo: scanner sweep + LDR, optionally reporting THREAT to the tank over LoRa.
void D9Test::comboThreat() {
    printHeader(F("THREAT DETECTION (findEnemy loop, darkest = lowest ldr @ angle)"));
    bool report = askYesNo(F("Send THREAT to TNK"), false);
    while (true) {
        startTest();
        Result result = scanForThreat(report);
        printResult(result);
        if (result != DONE) {
            return;
        }
        Serial.print(F("r + Enter scans again: "));
        char buffer[4];
        if (readLine(buffer, sizeof(buffer)) == 0 || (buffer[0] != 'r' && buffer[0] != 'R')) {
            return;
        }
    }
}

// Line-following combos: sensors + tracks + corrections, optionally with obstacle stop, marker stop and LoRa.
void D9Test::comboLine(const __FlashStringHelper* title, uint8_t flags) {
    printHeader(title);
    if (askYesNo(F("Tracks on (n = dry run, move it by hand)"), true)) {
        flags |= DRIVE_MOTORS;
    }
    uint8_t target = 0;
    if (flags & DRIVE_MARKER) {
        // 100 is IRReceiver's "nothing received" value, so it can't be a target.
        target = askNumber(F("Stop at marker"), 2, 0, NO_IR_CODE - 1);
    }
    if ((flags & DRIVE_LORA) && askYesNo(F("POSITION reports"), false)) {
        flags |= DRIVE_REPORTS;
    }
    Serial.println(F("* = on line. RIGHT = only left sensor on, left track fast. LOST keeps last PWM.\n"
                     "mix = loops straight/right/left/lost per line, hz = loops per second"));
    startTest();
    estimatedDistance = 0;
    Result result = followLine(flags, target);
    printResult(result);
}

// Runs the whole NORTH or SOUTH route step by step, with every subsystem logging.
void D9Test::comboMission() {
    printHeader(F("MISSION REHEARSAL"));
    // Same steps as northLoop() and southLoop() in mainD9.cpp.
    static const uint8_t northRoute[] = {2, SCAN_STEP, 4};
    static const uint8_t southRoute[] = {11, SCAN_STEP, 13, SCAN_STEP, TOW_STEP, 16};
    bool north = askNumber(F("1 NORTH, 2 SOUTH"), 1, 1, 2) == 1;
    uint8_t flags = DRIVE_OBSTACLE | DRIVE_MARKER | DRIVE_LORA;
    if (askYesNo(F("Tracks on"), true)) {
        flags |= DRIVE_MOTORS;
    }
    bool reports = askYesNo(F("LoRa reports"), false);
    if (reports) {
        flags |= DRIVE_REPORTS;
    }
    const uint8_t* route = north ? northRoute : southRoute;
    uint8_t steps = north ? sizeof(northRoute) : sizeof(southRoute);

    startTest();
    estimatedDistance = 0;
    Result result = DONE;
    for (uint8_t i = 0; i < steps && result == DONE; i++) {
        unsigned long stepStart = millis();
        logBegin(F("STEP"));
        if (route[i] == SCAN_STEP) {
            Serial.println(F(" scan"));
            // Like findEnemy(), a scan that finds nothing in time lets the route continue.
            result = scanForThreat(reports);
            if (result == TIMED_OUT) {
                result = DONE;
            }
        } else if (route[i] == TOW_STEP) {
            Serial.println(F(" tow"));
            if (ensureServo()) {
                d9.tow();
            } else {
                result = NOT_RUN;
            }
        } else {
            logField(F("toMarker"), route[i]);
            Serial.println();
            result = followLine(flags, route[i]);
        }
        logBegin(F("STEP"));
        logField(F("ms"), millis() - stepStart);
        Serial.println();
    }
    stopTracks();
    if (result == DONE && reports) {
        sendWithAck(F("HML"), F("FIN"), 0, 0, 2000);
    }
    printResult(result);
}

// Shows every sensor on one line without moving anything; IR codes and LoRa messages print as they arrive.
void D9Test::sensorDashboard() {
    printHeader(F("DASHBOARD"));
    EnemyDetector& detector = d9.getEnemyDetector();
    startTest();
    while (!stopRequested()) {
        readMarker();
        String message = genericPlatform.listen();
        if (message.length() > 0) {
            logBegin(F("LORA rx"));
            Serial.print(' ');
            Serial.println(message);
        }
        if (logDue()) {
            uint16_t leftValue;
            uint16_t rightValue;
            int8_t state = lineState(leftValue, rightValue);
            float distance = genericPlatform.checkSonicDistance();
            bool detected = detector.detected();
            logBegin(F("DASH"));
            logLineSensor(F("irL"), leftValue);
            logLineSensor(F("irR"), rightValue);
            printLineState(state);
            logDistance(distance);
            logField(F("ldr"), detector.getLastReading());
            if (detected) {
                Serial.print(F(" THREAT"));
            }
            if (servoReady) {
                logField(F("servo"), d9.getServoScanner().getAngle());
            }
            Serial.println();
        }
    }
}

// Changes log speed, the motion safety timeout and drive tuning without reflashing.
void D9Test::editOptions() {
    printHeader(F("OPTIONS (Enter keeps)"));
    logIntervalMs = askNumber(F("Log every ms"), logIntervalMs, 20, 10000);
    motionTimeoutS = askNumber(F("Motion timeout s"), motionTimeoutS, 1, 600);
    basePwm = askNumber(F("Base PWM"), basePwm, 0, 255);
    correctionPwm = askNumber(F("Steering slow-track PWM"), correctionPwm, 0, 255);
    lineThreshold = askNumber(F("Line threshold"), lineThreshold, 0, 1023);
    obstacleCm = askNumber(F("Obstacle cm"), obstacleCm, 1, 255);
}

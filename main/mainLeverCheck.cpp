// Front-lever servo check (servo on D9): finds out whether it is positional (goes to an angle and
// holds) or continuous (turns while it gets a signal), in small, slow steps.
// Copy this file over src/main.cpp, upload, open the Serial Monitor at 9600 and keep USB plugged in.
// The servo needs the battery (6 V servo rail); unplug the battery at any time to stop it.
// The tracks are held off.
//
// The servo is limp until a command and goes limp again right after. Type a command + Enter
// (several in one line run one after another, e.g. "++++"):
//   +  / -   pulse +10 / -10 us, then a short burst at the new pulse
//   g        short burst at the current pulse (BURST_MS)
//   h        hold the current pulse for 1 s - the key test, see below
//   c        set the pulse back to 1500 us (center / stop) and send a short burst
//   p        print the pulse and what it would mean for each servo type
//
// How to tell the type, starting from 1500 us:
//   Positional: each + or - moves the lever a little and it stays there; "h" moves it to a spot
//               and then it holds still for the rest of the second.
//   Continuous: at 1500 it stays still (or creeps slightly); away from 1500, "h" keeps it turning
//               for the whole second and it only stops when the signal ends.
// Note: on a positional servo, the very first burst moves the lever toward the angle for the
// current pulse (1500 = middle of its travel). Bursts are short so that first move is gradual.
#include <Arduino.h>
#include <Servo.h>

const uint8_t LEVER_PIN = 9;
const int PULSE_MIN = 500;            // software limits, wide enough for 180/270/360 servos
const int PULSE_MAX = 2500;
const int PULSE_STEP = 10;            // about 1.4 deg on a 270-degree servo, 1 deg on a 180
const unsigned long BURST_MS = 60;    // three servo pulses
const unsigned long HOLD_MS = 1000;

// Track speed pins, only used to keep the motors off.
const uint8_t LEFT_EN = 6;
const uint8_t RIGHT_EN = 5;

Servo lever;
int pulse = 1500;

// Prints "[seconds] " before a log line.
void logTime() {
  Serial.print('[');
  Serial.print(millis() / 1000.0f, 1);
  Serial.print(F("s] "));
}

// Prints the pulse and how each servo type would read it.
void printPulse() {
  logTime();
  Serial.print(F("pulse="));
  Serial.print(pulse);
  Serial.print(F("us  as 270-deg: "));
  Serial.print((pulse - 500) * 270L / 2000);
  Serial.print(F(" deg  as 180-deg: "));
  Serial.print((long)(pulse - 544) * 180L / (2400 - 544));
  Serial.print(F(" deg  as continuous: "));
  if (abs(pulse - 1500) < 20) {
    Serial.println(F("stop"));
  } else {
    Serial.print(pulse < 1500 ? F("turn one way, speed ") : F("turn other way, speed "));
    Serial.println(abs(pulse - 1500));
  }
}

// Sends the current pulse for ms milliseconds, then goes limp.
void sendFor(unsigned long ms) {
  lever.writeMicroseconds(pulse);
  lever.attach(LEVER_PIN, PULSE_MIN, PULSE_MAX);
  delay(ms);
  lever.detach();
}

void setup() {
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
  Serial.begin(9600);
  Serial.println(F("Front-lever servo check (D9). Servo is limp until a command."));
  Serial.println(F("+ / - step 10us | g burst | h hold 1 s | c back to 1500 | p print"));
  Serial.println(F("Start: c (a few times), then h. Watch whether it moves and holds, or keeps turning."));
  printPulse();
}

void loop() {
  if (!Serial.available()) {
    return;
  }
  char c = Serial.read();
  if (c == '+' || c == '-') {
    pulse = constrain(pulse + (c == '+' ? PULSE_STEP : -PULSE_STEP), PULSE_MIN, PULSE_MAX);
    sendFor(BURST_MS);
    printPulse();
  } else if (c == 'g') {
    sendFor(BURST_MS);
    printPulse();
  } else if (c == 'h') {
    logTime();
    Serial.println(F("Holding for 1 s..."));
    sendFor(HOLD_MS);
    printPulse();
  } else if (c == 'c') {
    pulse = 1500;
    sendFor(BURST_MS);
    printPulse();
  } else if (c == 'p') {
    printPulse();
  }
}

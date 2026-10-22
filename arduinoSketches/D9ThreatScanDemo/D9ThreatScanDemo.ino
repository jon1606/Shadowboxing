// D9 threat-scan demo. Runs on its own: upload, unplug USB, power from the battery
// (the servo needs the 6 V servo rail). The tracks are held off the whole time.
// Same sweep and detection as the mission code (ServoScanner + EnemyDetector).
//
// The radar (a 270-degree servo) sweeps 0-180 deg. When the laser hits the LDR it stops and points there.
// Board LED (D13):
//   off         = scanning
//   on          = threat found, holding that angle for 3 s
//   fast blink  = waiting for the laser to go away before scanning again

#include <Servo.h>

const uint8_t SERVO_PIN = 3;   // radar servo
const uint8_t LDR_PIN = A3;    // 10k to 5V, LDR to GND: more light = lower reading
const uint8_t LED = LED_BUILTIN;

// Track speed pins, only used to keep the motors off.
const uint8_t LEFT_EN = 6;
const uint8_t RIGHT_EN = 5;

// Tuning, same values as mainD9.cpp / ServoScanner.
const int LASER_THRESHOLD = 300;          // a reading below this = laser on the LDR
const uint8_t MIN_ANGLE = 0;
const uint8_t MAX_ANGLE = 180;
const unsigned long STEP_MS = 17;         // 1 deg per step, about 3 s per sweep
const unsigned long HOLD_MS = 3000;       // how long to point at a threat
const unsigned long CLEAR_MS = 500;       // laser must be gone this long before scanning again

// 270-degree servo: pulse widths at 0 and 270 deg.
const int SERVO_RANGE = 270;
const int PULSE_MIN = 500;
const int PULSE_MAX = 2500;

enum State { SCANNING, HOLDING, WAIT_CLEAR };

Servo radar;
State state = SCANNING;
uint8_t angle = MIN_ANGLE;
int8_t direction = 1;
unsigned long lastStep = 0;
unsigned long stateSince = 0;
unsigned long clearSince = 0;
uint16_t threats = 0;

// Moves the radar to a real angle (0-270).
void moveTo(int degrees) {
  radar.writeMicroseconds(map(degrees, 0, SERVO_RANGE, PULSE_MIN, PULSE_MAX));
}

void enterState(State next) {
  state = next;
  stateSince = millis();
}

void setup() {
  Serial.begin(9600);
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
  pinMode(LDR_PIN, INPUT);
  pinMode(LED, OUTPUT);

  radar.attach(SERVO_PIN, PULSE_MIN, PULSE_MAX);
  moveTo(angle);
  Serial.print(F("D9 threat scan demo. LDR now: "));
  Serial.print(analogRead(LDR_PIN));
  Serial.print(F(" (threat below "));
  Serial.print(LASER_THRESHOLD);
  Serial.println(F(")"));
  delay(500);
  enterState(SCANNING);
}

void loop() {
  unsigned long now = millis();
  int light = analogRead(LDR_PIN);

  if (state == SCANNING) {
    digitalWrite(LED, LOW);
    if (light < LASER_THRESHOLD) {
      threats++;
      Serial.print(F("THREAT #"));
      Serial.print(threats);
      Serial.print(F(" at angle "));
      Serial.print(angle);
      Serial.print(F(", ldr "));
      Serial.println(light);
      enterState(HOLDING);
    } else if (now - lastStep >= STEP_MS) {
      lastStep = now;
      // Back-and-forth sweep, like ServoScanner::update().
      if (angle >= MAX_ANGLE) {
        direction = -1;
      } else if (angle <= MIN_ANGLE) {
        direction = 1;
      }
      angle += direction;
      moveTo(angle);
    }
  } else if (state == HOLDING) {
    digitalWrite(LED, HIGH);
    if (now - stateSince >= HOLD_MS) {
      clearSince = now;
      enterState(WAIT_CLEAR);
    }
  } else {
    digitalWrite(LED, (now / 100) % 2);
    if (light < LASER_THRESHOLD) {
      clearSince = now;
    } else if (now - clearSince >= CLEAR_MS) {
      Serial.println(F("Clear, scanning again"));
      enterState(SCANNING);
    }
  }
}

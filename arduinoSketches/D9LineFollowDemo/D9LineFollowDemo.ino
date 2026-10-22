// D9 line-following demo. Runs on its own: upload, unplug USB, power from the battery.
// Same pins and steering as the mission code (GenericPlatform::updateDrive()).
//
// Board LED (D13):
//   slow blink  = start countdown (5 s), put the D9 on the line
//   on          = following the line
//   fast blink  = line lost for 1.5 s, tracks stopped (it restarts if the line comes back)

// L298N: each track has its own speed pin; both tracks share the direction pins.
const uint8_t LEFT_EN = 6;     // ENA
const uint8_t RIGHT_EN = 5;    // ENB
const uint8_t DIR_A = 7;       // IN1 + IN3
const uint8_t DIR_B = 8;       // IN2 + IN4

// Line sensors (analog out).
const uint8_t LEFT_SENSOR = A5;
const uint8_t RIGHT_SENSOR = A4;

const uint8_t LED = LED_BUILTIN;

// Tuning, same values as mainD9.cpp / GenericPlatform.cpp.
const int LINE_THRESHOLD = 300;              // a sensor reading above this counts as "on"
const int BASE_PWM = 200;                    // speed when driving straight
const int TURN_PWM = 0;                      // inner track while steering
const unsigned long START_DELAY_MS = 5000;
const unsigned long LOST_TIMEOUT_MS = 1500;  // keep the last command this long, then stop
const unsigned long LOG_INTERVAL_MS = 250;   // Serial log rate (only matters with USB plugged in)

unsigned long lastSeen = 0;
unsigned long lastLog = 0;
int leftPwm = 0;
int rightPwm = 0;

// Sets both track speeds. Direction never changes: forward is IN1/IN3 HIGH, IN2/IN4 LOW.
void drive(int left, int right) {
  leftPwm = left;
  rightPwm = right;
  analogWrite(LEFT_EN, left);
  analogWrite(RIGHT_EN, right);
}

void setup() {
  Serial.begin(9600);
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LED, OUTPUT);
  drive(0, 0);
  digitalWrite(DIR_A, HIGH);  // swapped: LOW/HIGH drove the D9 backwards
  digitalWrite(DIR_B, LOW);

  Serial.println(F("D9 line follow demo: starting in 5 s"));
  unsigned long start = millis();
  while (millis() - start < START_DELAY_MS) {
    digitalWrite(LED, (millis() / 500) % 2);
  }
  lastSeen = millis();
}

void loop() {
  int leftValue = analogRead(LEFT_SENSOR);
  int rightValue = analogRead(RIGHT_SENSOR);
  bool left = leftValue > LINE_THRESHOLD;
  bool right = rightValue > LINE_THRESHOLD;
  unsigned long now = millis();
  bool lost = false;

  // Same decisions as updateDrive(): both on = straight, one on = slow the other track.
  if (left && right) {
    drive(BASE_PWM, BASE_PWM);
    lastSeen = now;
  } else if (left) {
    drive(BASE_PWM, TURN_PWM);
    lastSeen = now;
  } else if (right) {
    drive(TURN_PWM, BASE_PWM);
    lastSeen = now;
  } else if (now - lastSeen >= LOST_TIMEOUT_MS) {
    drive(0, 0);
    lost = true;
  }

  digitalWrite(LED, lost ? (now / 100) % 2 : HIGH);

  if (now - lastLog >= LOG_INTERVAL_MS) {
    lastLog = now;
    Serial.print(F("L="));
    Serial.print(leftValue);
    Serial.print(left ? F("* R=") : F(" R="));
    Serial.print(rightValue);
    Serial.print(right ? F("* pwm=") : F(" pwm="));
    Serial.print(leftPwm);
    Serial.print('/');
    Serial.print(rightPwm);
    Serial.println(lost ? F(" LOST, stopped") : F(""));
  }
}

#include "TrackPilot.h"

// Track speed when driving straight, as in GenericPlatform.cpp.
#define BASE_PWM 200

// A correction starts gently (inner track slowed) so small drifts don't zig-zag, and turns hard once
// it has lasted SOFT_TURN_MS (inner track stopped, outer at full power: at 200 it stalled mid-turn).
#define SOFT_TURN_MS 150
#define SOFT_TURN_INNER_PWM 120
#define HARD_TURN_INNER_PWM 0
#define HARD_TURN_OUTER_PWM 255

// From standstill the tracks may not break free (especially in a pivot), so driving starts with a
// short straight push at full power.
#define KICK_PWM 255
#define KICK_MS 200

// The path is sampled every PATH_SAMPLE_MS and the recording keeps the last PATH_SAMPLES samples
// (7 s). Reversing past the recorded part goes straight back at REVERSE_PWM.
#define PATH_SAMPLE_MS 50
#define PATH_SAMPLES 140
#define REVERSE_PWM 200

// GenericPlatform::readLine() decisions.
#define LINE_STRAIGHT 0
#define LINE_STEER_RIGHT 1
#define LINE_STEER_LEFT -1

// The recording lives here rather than in the object, so programs that never drive with a TrackPilot
// (the test bench, the demo) don't pay its 280 bytes out of the Arduino's 2 KB of RAM.
static uint8_t pathLeft[PATH_SAMPLES];
static uint8_t pathRight[PATH_SAMPLES];
static uint8_t pathEnd = 0;          // where the next sample goes; the newest one is just before it
static uint8_t pathLength = 0;
static unsigned long lastSampleTime = 0;

TrackPilot::TrackPilot(GenericPlatform& basePlatform)
    : genericPlatform(basePlatform),
      steering(0),
      correctionStart(0),
      leftPwm(0),
      rightPwm(0) {}

void TrackPilot::begin() {
    steering = 0;
    pathLength = 0;
}

// Pushes off from standstill. Time spent stopped isn't part of the recorded path.
void TrackPilot::start() {
    drive(KICK_PWM, KICK_PWM);
    delay(KICK_MS);
    lastSampleTime = millis();
}

// One line-following step: steers toward the line and records the path.
void TrackPilot::followLine() {
    int8_t line = genericPlatform.readLine();
    unsigned long now = millis();
    if (line == LINE_STEER_RIGHT || line == LINE_STEER_LEFT) {
        if (line != steering) {
            steering = line;
            correctionStart = now;
        }
    } else if (line == LINE_STRAIGHT) {
        steering = 0;
    }
    // Neither sensor on the road: keep the last command.

    bool hardTurn = now - correctionStart >= SOFT_TURN_MS;
    uint8_t inner = hardTurn ? HARD_TURN_INNER_PWM : SOFT_TURN_INNER_PWM;
    uint8_t outer = hardTurn ? HARD_TURN_OUTER_PWM : BASE_PWM;
    if (steering == LINE_STEER_RIGHT) {
        drive(outer, inner);
    } else if (steering == LINE_STEER_LEFT) {
        drive(inner, outer);
    } else {
        drive(BASE_PWM, BASE_PWM);
    }
    recordPath();
}

void TrackPilot::stop() {
    genericPlatform.stopDrive();
    leftPwm = 0;
    rightPwm = 0;
}

// Drives backwards for durationMs, replaying the recorded path newest first so the D9 retraces the
// curves it came along; once the recording runs out it reverses straight. The replayed part is used up.
void TrackPilot::reverseAlongPath(unsigned long durationMs) {
    unsigned long start = millis();
    while (millis() - start < durationMs) {
        uint8_t left = REVERSE_PWM;
        uint8_t right = REVERSE_PWM;
        if (pathLength > 0) {
            pathEnd = (pathEnd + PATH_SAMPLES - 1) % PATH_SAMPLES;
            pathLength--;
            left = pathLeft[pathEnd];
            right = pathRight[pathEnd];
        }
        genericPlatform.driveTracks(-left, -right);
        delay(PATH_SAMPLE_MS);
    }
    stop();
}

// Drives forward and remembers the speeds for the recording.
void TrackPilot::drive(uint8_t left, uint8_t right) {
    leftPwm = left;
    rightPwm = right;
    genericPlatform.driveTracks(left, right);
}

// Stores the track speeds every PATH_SAMPLE_MS, overwriting the oldest sample once the recording is full.
void TrackPilot::recordPath() {
    unsigned long now = millis();
    if (now - lastSampleTime < PATH_SAMPLE_MS) {
        return;
    }
    lastSampleTime = now;
    pathLeft[pathEnd] = leftPwm;
    pathRight[pathEnd] = rightPwm;
    pathEnd = (pathEnd + 1) % PATH_SAMPLES;
    if (pathLength < PATH_SAMPLES) {
        pathLength++;
    }
}

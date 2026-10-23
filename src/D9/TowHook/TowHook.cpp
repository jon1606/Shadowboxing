#include "TowHook.h"
#include "../../Generic/Sonic/Sonic.h"

// The FS90R stands still at TOW_STOP_US; the hook goes down below it and up above it.
#define TOW_STOP_US 1500
#define TOW_SPEED_US 60

// How long the hook turns to go all the way down and back up. Measured with mainTowTest.cpp
// (300 and 400 ms), then 200 ms added to each so it seats and lifts firmly.
#define TOW_DOWN_MS 500
#define TOW_UP_MS 600

// Marks savedState as written by this program rather than the random contents of RAM at power-on.
#define SAVED_STATE_MAGIC 0xD9A57A7EUL

// Whether the hook is down, kept in .noinit RAM: a reset leaves it alone and only a power cut clears
// it, so begin() can tell that a reset (e.g. a battery dip while towing) left the hook down.
struct SavedState {
    uint32_t magic;
    bool down;
};

static SavedState savedState __attribute__((section(".noinit")));

TowHook::TowHook(uint8_t pin)
    : servoPin(pin),
      interrupted(false) {}

// Lifts the hook if a reset left it down, otherwise leaves it alone: lifting a hook that is already
// up pushes it against its stop, which can reset the Arduino when the servo runs off its 5 V.
void TowHook::begin() {
    interrupted = savedState.magic == SAVED_STATE_MAGIC && savedState.down;
    if (interrupted) {
        lift();
    } else {
        saveDown(false);
    }
}

void TowHook::lower() {
    saveDown(true);
    turn(-TOW_SPEED_US, TOW_DOWN_MS);
}

void TowHook::lift() {
    turn(TOW_SPEED_US, TOW_UP_MS);
    saveDown(false);
}

// True if begin() found the hook left down by a reset, i.e. the last tow was cut short.
bool TowHook::wasInterrupted() const {
    return interrupted;
}

// Turns the servo at speedUs from the stop pulse for durationMs, then stops it and lets it go limp.
// A hook on NO_PIN isn't fitted and never moves.
void TowHook::turn(int speedUs, unsigned long durationMs) {
    if (servoPin == NO_PIN) {
        return;
    }
    servo.writeMicroseconds(TOW_STOP_US + speedUs);
    servo.attach(servoPin);
    delay(durationMs);
    servo.writeMicroseconds(TOW_STOP_US);
    delay(40);
    servo.detach();
}

void TowHook::saveDown(bool down) {
    savedState.magic = SAVED_STATE_MAGIC;
    savedState.down = down;
}

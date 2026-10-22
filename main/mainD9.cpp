#include "Generic/GenericPlatform/GenericPlatform.h"
#include <Arduino.h>
#include "D9/D9/D9.h"

// Keeps the program waiting for the mission selection command.
bool missionNotStarted = true;

// Keeps the program waiting for the command to begin movement.
bool MoveNotStarted = true;

// Identifies the two possible mission routes.
enum Mission {
  NORTH,
  SOUTH
} mission;

// Left motor: IN1=8, IN2=7, EN=6 (7 and 8 swapped in code so positive speed drives forward)
// Right motor: IN1=8, IN2=7, EN=5
// IR sensors: left=A5, right=A4, LoRa: module TXD -> D2, module RXD <- D10
// No ultrasonic sensor on the D9, IR receiver: 4
// Servos: radar D3, front lever D9, tow D11 (lever and tow have no code yet)
// The threshold for line detection is set to 300 (for analog sensors).

GenericPlatform basePlatform(
  8, 7, 6, //Left motor (7/8 swapped: the D9 drove backwards)
  8, 7, 5, //Right motor
  A5, A4, 2, 10, //Ir analog and Lora
  NO_PIN, NO_PIN, 4, //No ultrasonic, IR receiver
  300 , "D9" //Threshold and LoRa name (HAMAL addresses "D9")
);

// Add the D9-specific sensors and actuators to the shared platform.
D9Platform d9(
    basePlatform,
    A3, // LDR pin
    300, // Laser threshold
    3 // Radar servo pin (D9 is the front lever servo)
);

void northLoop();
void southLoop();

// Initializes serial diagnostics and all platform hardware.
void setup() {
    Serial.begin(9600);
    d9.begin();
}

void loop() {

    // Wait until the hamal selects the north or south mission.
    while(missionNotStarted) {
      String command = d9.getGenericPlatform().listen();
      if (d9.getGenericPlatform().needToStop()) {
        d9.getGenericPlatform().endMission();
      }
      String type = command.substring(0, command.indexOf('|'));
      if (type == "NORTH") {
        mission = NORTH;
        missionNotStarted = false;
      } else if (type == "SOUTH") {
        mission = SOUTH;
        missionNotStarted = false;
      }
    }

    // Wait until the hamal authorizes the platform to start moving.
    while(MoveNotStarted) {
      String command = d9.getGenericPlatform().listen();
      if (d9.getGenericPlatform().needToStop()) {
        d9.getGenericPlatform().endMission();
        break;
      }
      String type = command.substring(0, command.indexOf('|'));
      if (type == "GO") {
        MoveNotStarted = false;
      }
    }

    // Run the route selected during the mission setup phase.
    if (mission == NORTH) {
        northLoop();
    } else if (mission == SOUTH) {
        southLoop();
    }
}

void northLoop() {
  d9.getGenericPlatform().driveUntilFoundIR(2);
  d9.findEnemy();
  d9.getGenericPlatform().driveUntilFoundIR(4);
  d9.getGenericPlatform().endMission();
}

void southLoop() {
    d9.getGenericPlatform().driveUntilFoundIR(11);
    d9.findEnemy();
    d9.getGenericPlatform().driveUntilFoundIR(13);
    d9.findEnemy();
    d9.tow();
    d9.getGenericPlatform().driveUntilFoundIR(16);
    d9.getGenericPlatform().endMission();
}

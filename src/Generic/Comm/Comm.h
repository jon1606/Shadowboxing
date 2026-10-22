
#ifndef COMM_H
#define COMM_H

#include <Arduino.h>
#include <SoftwareSerial.h>

// Longest line listen() accepts; longer lines are dropped. Messages today are about 30 characters.
#define COMM_LINE_SIZE 64

class Comm {
private:
  SoftwareSerial loraSerial;
  String sender;

  // The line being received, kept between listen() calls so listen() never has to wait.
  char line[COMM_LINE_SIZE];
  uint8_t lineLength;
  bool lineTooLong;

  // Counter of the last ACK addressed to us, so sendMessage() only accepts the ACK for its own message.
  uint16_t lastAckCounter;
  bool stopSeen;

  String handleLine();

public:
  Comm(
    uint8_t txdPin,
    uint8_t rxdPin,
    String sender
  );

  void begin();
  String listen();
  bool sendMessage(String to, String type, int x, int angle, int counter, bool waitForAck = true);
  bool stopReceived();
  SoftwareSerial GetLoraSerial();
};

#endif

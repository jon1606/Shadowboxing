#include "Comm.h"

// How long sendMessage() waits for an ACK before sending the message again.
#define ACK_TIMEOUT_MS 1500UL

// How many times sendMessage() resends an unacknowledged message before giving up.
#define MAX_RETRIES 3

// A partial line with no new bytes for this long is handled as complete (readStringUntil's timeout).
#define LINE_TIMEOUT_MS 1000UL

// Initializes the software serial connection and identifies this device as the sender.
Comm::Comm(
  uint8_t txdPin,
  uint8_t rxdPin,
  String name

)
  : loraSerial(txdPin, rxdPin),
    sender(name),
    lineLength(0),
    lineTooLong(false),
    lastByteAt(0),
    lastAckCounter(0),
    stopSeen(false)
{}

// Starts the LoRa serial link
void Comm::begin() {
  loraSerial.begin(9600);
}

// Serializes a message as sender|recipient|type|x|angle|counter and transmits it.
// Unless waitForAck is false, resends it (same counter) until its ACK arrives; returns false if
// no ACK came after MAX_RETRIES resends or a STOP arrived meanwhile.
bool Comm::sendMessage(String to, String type, int x, int angle, int counter, bool waitForAck) {
  String msg = (String)sender + "|" +
               (String)to + "|" +
               (String)type + "|" +
               String(x) + "|" +
               String(angle) + "|" + String(counter);
  if (type == "ACK" || !waitForAck) {
    loraSerial.println(msg);
    return true;
  }

  for (uint8_t attempt = 0; attempt <= MAX_RETRIES; attempt++) {
    loraSerial.println(msg);
    unsigned long sentAt = millis();
    while (millis() - sentAt < ACK_TIMEOUT_MS) {
      String reply = listen();
      if (reply == "STOP") {
        return false;
      }
      if (reply.startsWith("ACK") && lastAckCounter == (uint16_t)counter) {
        return true;
      }
    }
  }
  return false;
}

// Reads whatever bytes have arrived without waiting; when a full line is in, acknowledges it if it
// is addressed to us and returns its type|x|angle. Returns "" while no complete message is ready.
String Comm::listen() {
  while (loraSerial.available()) {
    char c = loraSerial.read();
    lastByteAt = millis();
    // A line ends at '\n' or '\r' (so "\r\n", "\n" and "\r" senders all work).
    if (c != '\n' && c != '\r') {
      if (lineLength < COMM_LINE_SIZE - 1) {
        line[lineLength++] = c;
      } else {
        lineTooLong = true;
      }
      continue;
    }
    String result = finishLine();
    if (result.length() > 0) {
      return result;
    }
  }

  // Like the old readStringUntil(), a line that stops arriving without an end counts as complete.
  if ((lineLength > 0 || lineTooLong) && millis() - lastByteAt >= LINE_TIMEOUT_MS) {
    return finishLine();
  }

  // Report that no complete message is currently available.
  return "";
}

// Closes the line being received and handles it; empty and over-long lines give "".
String Comm::finishLine() {
  line[lineLength] = '\0';
  bool dropLine = lineTooLong || lineLength == 0;
  lineLength = 0;
  lineTooLong = false;
  if (dropLine) {
    return "";
  }
  return handleLine();
}

// Parses the completed line (sender|to|type|x|angle|counter), ACKs it when it is for us,
// and returns what listen() should report.
String Comm::handleLine() {
  // Split on '|' into the six protocol fields; missing fields stay empty.
  String fields[6];
  uint8_t field = 0;
  for (uint8_t i = 0; line[i] != '\0'; i++) {
    if (line[i] == '|') {
      if (field < 5) {
        field++;
      }
    } else {
      fields[field] += line[i];
    }
  }
  const String& from = fields[0];
  const String& to = fields[1];
  const String& type = fields[2];

  // A STOP is obeyed whoever it is addressed to.
  if (type == "STOP") {
    stopSeen = true;
    return "STOP";
  }

  uint16_t x = fields[3].toInt();
  int16_t angle = fields[4].toInt();
  uint16_t counter = fields[5].toInt();

  // Ignore messages intended for another device while accepting broadcast messages.
  if (to != this->sender && to != "ALL") {
    return "";
  }

  if (type == "ACK") {
    lastAckCounter = counter;
  } else {
    // Confirm receipt using the original message values and return its message type.
    sendMessage(from, "ACK", x, angle, counter);
  }

  return type + "|" + String(x) + "|" + String(angle);
}

// Returns true once any STOP has been received, even one consumed while waiting for an ACK.
bool Comm::stopReceived() {
  return stopSeen;
}

SoftwareSerial Comm::GetLoraSerial() {
  return loraSerial;
}

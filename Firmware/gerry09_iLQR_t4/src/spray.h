/**
 * This controls the spray paint actuator over bluetooth.  This code only
 * handles the bluetooth/Teensy side and not the code for the remote
 * micronotroller physically attached to the spray can + servo.
 */

#pragma once

#include <Stream.h>

#include "communication/ascii_parser.h"
#include "communication/cdpr_serial_commands.h"

class Spray {
 public:
  Spray(HardwareSerial& serial) : serial_(serial), on_(false) {}

  // Common API
  void setup() { serial_.begin(9600); }
  void update() {
    if (send_timer_.check()) {
      serial_.write(on_ ? on_value_ : off_value_);
    }
  }

  void changeColor(int color) {
    serial_.write('c');
    serial_.println(color, DEC);
  }
  void setSpray(bool on) { on_ = on; }
  void setSprayOnValue(uint8_t on_value) { on_value_ = on_value; }
  void setSprayOffValue(uint8_t off_value) { off_value_ = off_value; }
  bool spray() const { return on_; }

  // Serial API
  bool parseMsg(AsciiParser parser, Stream& serial);
  void forward_msg(char* buf, int len) { serial_.write(buf, len); }

 private:
  HardwareSerial& serial_;
  bool on_;
  uint8_t off_value_{'0'};
  uint8_t on_value_{'1'};
  Metro send_timer_{50};
};

bool Spray::parseMsg(AsciiParser parser, Stream& serial) {
  UNWRAP_PARSE_CHECK(, parser.checkChar(SerialPrefixes::SPRAY));
  // First check for 0 and 1, for backwards compatibility.
  AsciiParser parser_backup = parser;
  UNWRAP_PARSE_CHECK(char val, parser.getChar(&val));
  switch (val) {
    case '0':
      serial.println("Spray off");
      setSpray(false);
      return true;
    case '1':
      serial.println("Spray on");
      setSpray(true);
      return true;
    case '>': {
      UNWRAP_PARSE_CHECK(uint8_t val, parser.parseInt(&val));
      if (!parser.checkDone()) return false;
      if (val > 180) return false;
      setSprayOffValue(val + 65);  // +65 to avoid collision with ASCII
      serial.printf("Set spray off value to %d\n", val);
      return true;
    }
    case '<': {
      UNWRAP_PARSE_CHECK(uint8_t val, parser.parseInt(&val));
      if (!parser.checkDone()) return false;
      if (val > 180) return false;
      setSprayOnValue(val + 65);  // +65 to avoid collision with ASCII
      serial.printf("Set spray on value to %d\n", val);
      return true;
    }
    default: {
      // Forward rest of args to spray paint MCU
      serial.println("Couldn't parse. Forwarding to spray paint MCU.");
      forward_msg(parser_backup.get_buffer_cur(), parser_backup.len());
      return true;
    }
  }
}

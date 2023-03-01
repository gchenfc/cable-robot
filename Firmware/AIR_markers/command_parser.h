#pragma once

#include "end_effector.h"

class SerialParser {
 public:
  enum State { INITIAL_COMMAND, ARGUMENT };
  enum Commands {
    MOVE,
    MOVETO,
    SPEED,
    COLOR,
  };

 private:
  char buffer[300];
  uint64_t buffer_index;
  float argument_buffer_float[10];
  int argument_buffer_int[10];
  uint8_t argument_buffer_float_index;
  uint8_t argument_buffer_int_index;
  State state_;
  Commands command_;
  Stream &serial_in_, &serial_out_;
  EndEffector &end_effector_;

 public:
  SerialParser(Stream &serial_in, Stream &serial_out, EndEffector &end_effector)
      : serial_in_(serial_in),
        serial_out_(serial_out),
        end_effector_(end_effector) {}

  void update() {
    if (serial_in_.available()) {
      char c = serial_in_.read();
      Serial.print(c);
      switch (state_) {
        case INITIAL_COMMAND:
          parse_command_code(c);
          break;
        case ARGUMENT:
          parse_arg(c);
          break;
      }
    }
  }

  void parse_command_code(char c) {
    switch (c) {
      case 'm':
        command_ = MOVE;
        break;
      case 'M':
        command_ = MOVETO;
        break;
      case 's':
        command_ = SPEED;
        break;
      case 'c':
        command_ = COLOR;
        break;
      case '2':
        end_effector_.set_ignore_pen(true);
        break;
      case '3':
        end_effector_.set_ignore_pen(false);
        break;
      case '0':
        end_effector_.pen_in();
        return;
      case '1':
        end_effector_.pen_out();
        return;
      default:
        serial_out_.println("Parse error!");
        return;
    }
    state_ = ARGUMENT;
    buffer_index = 0;
    argument_buffer_float_index = 0;
    argument_buffer_int_index = 0;
  }

  void reset_parse() { state_ = INITIAL_COMMAND; }

  void parse_arg(char c);

  void parse_int() {
    buffer[buffer_index] = '\0';
    argument_buffer_int[argument_buffer_int_index++] = atoi(buffer);
    buffer_index = 0;
  }
  void parse_float() {
    buffer[buffer_index] = '\0';
    argument_buffer_float[argument_buffer_float_index++] = atof(buffer);
    buffer_index = 0;
  }

  void do_parse_action();
};

void SerialParser::do_parse_action() {
  switch (command_) {
    case MOVE:
      if (!end_effector_.move(argument_buffer_int[0],
                              argument_buffer_float[0])) {
        serial_out_.println("Invalid move!");
      }
      break;
    case MOVETO:
      if (!end_effector_.move_to(argument_buffer_int[0],
                                 argument_buffer_float[0])) {
        serial_out_.println("Invalid move!");
      }
      break;
    case SPEED:
      if (!end_effector_.set_rpm(argument_buffer_int[0],
                                 argument_buffer_float[0])) {
        serial_out_.println("Invalid speed!");
      }
      break;
    case COLOR:
      end_effector_.set_color(argument_buffer_int[0]);
      break;
  }
}

#define NUMBER                                                          \
  '0' : case '1' : case '2' : case '3' : case '4' : case '5' : case '6' \
      : case '7' : case '8' : case '9' : case '-' : case '.'

void SerialParser::parse_arg(char c) {
  switch (c) {
    case NUMBER:
      buffer[buffer_index++] = c;
      break;
    case ',':  // parse one arg
      parse_int();
      break;
    case ';':
    case '\n':  // parse last arg
      if (command_ == COLOR) {
        parse_int();
      } else {
        parse_float();
      }
      serial_out_.print("Parsed ");
      serial_out_.print(command_);
      serial_out_.println(": ");
      for (int i = 0; i < argument_buffer_int_index; i++) {
        serial_out_.print("\t");
        serial_out_.print(argument_buffer_int[i], DEC);
      }
      serial_out_.println();
      for (int i = 0; i < argument_buffer_float_index; i++) {
        serial_out_.print("\t");
        serial_out_.print(argument_buffer_float[i]);
      }
      serial_out_.println();
      do_parse_action();
      reset_parse();
      break;
    default:
      serial_out_.println("Parse error!");
      return;
  }
}

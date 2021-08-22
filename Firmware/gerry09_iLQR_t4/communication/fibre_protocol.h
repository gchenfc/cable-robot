/**
 * This file handles the odrive "native", "fibre" communication protocol
 * detailed here:
 * https://docs.odriverobotics.com/protocol
 *
 * One note that isn't detailed in the docs: reading from JSON (endpoint 0): the
 * argument is the byte number to start reading from.  The special argument "-1"
 * (0xFFFFFFFF) gets the crc (quick check if we have the same protocol).  You
 * need to request 4 bytes - the "first" 2 bytes are the json crc.  I believe
 * the "last" 2 bytes are the "json version id" which is the crc16 of the JSON
 * again, but using the original json crc as the "initial" value.
 *
 * TODO:
 *  - Receiving (note: packet is in reverse order)
 *  - CRC check received messages
 *  - Seq number
 *  - Wrap in a class / namespace
 */

#pragma once

#include <cstring>  // for memcpy

#include "arduino_test_utils.h"
#include "communication/crc.h"
#include "communication/fibre_endpoints.h"

// Constants
constexpr static uint8_t SYNC = 0xAA;
static constexpr uint16_t JSON_CRC_16 = 0x9b40;

// Extra endpoints that aren't defined in fibre_endpoints.h
namespace odrive {
enum {
  JSON = 0,
};
template <>
struct endpoint_type<JSON> {
  typedef uint32_t type;
};
}  // namespace odrive

// Declarations
static uint16_t seq_num = 0x0081;
/**
 * Send a message expecting a response
 * @tparam ENDPOINT The endpoint ID
 * @param response_size The size of the expected response from the odrive
 * @param payload The "function argument"
 * @param serial The serial port to send this data
 */
template <uint16_t ENDPOINT>
void sendMsg(const uint16_t response_size,
             const odrive::endpoint_type_t<ENDPOINT>& payload, Print* serial);
/**
 * Send a message that isn't expecting data back, but might request an ACK
 * @tparam ENDPOINT The endpoint ID
 * @tparam ACK Whether we want an acknowledgement response or not
 * @param response_size The size of the expected response from the odrive
 * @param payload The "function argument"
 * @param serial The serial port to send this data
 */
template <uint16_t ENDPOINT, bool ACK>
void sendMsg(const odrive::endpoint_type_t<ENDPOINT>& payload, Print* serial);
/**
 * Dynamic version of the previous two `sendMsg` functions.  Slightly more
 * dangerous.
 */
template <typename T>
void sendMsg(const uint16_t endpoint, const uint16_t response_size,
             const T& payload, Print* serial);
/**
 * Send a message as a raw byte buffer.  Even more dangerous.
 */
void sendMsgRaw(const uint16_t endpoint, const uint16_t response_size,
                const uint8_t payload[], const size_t payload_size,
                Print* serial);
/**
 * Populates the packet byte buffer given the payload and metadata
 */
uint8_t fillPacket(const uint16_t seq_num, const uint16_t endpoint,
                   const uint16_t response_size, const uint8_t payload[],
                   const size_t payload_size, uint8_t packet[]);
/**
 * Populates the stream byte buffer given the packet.
 */
uint16_t fillStream(const uint8_t packet[], const uint8_t packet_size,
                    uint8_t stream[]);

/**
 * This class is to help debug communication.  If you "sendMsg" to a DebugSerial
 * object instead of a normal Serial object, it'll print out the message you
 * tried to send in human-readable format.
 */
class DebugSerial : public Print {
 public:
  DebugSerial(Print* serial) : serial_(serial) {}

  size_t write(const uint8_t* buf, size_t len) override {
    serial_->print(millis());
    serial_->print(" Sent a message:\t");
    for (size_t i = 0; i < len; ++i) {
      sprintf(hexCar, "%02X ", buf[i]);
      serial_->print(hexCar);
    }
    serial_->println();
    return len;
  }

  size_t write(uint8_t b) override {
    serial_->print(millis());
    serial_->print(" Sent a byte: ");
    sprintf(hexCar, "%02X ", b);
    serial_->println(hexCar);
    return 1;
  };

 private:
  Print* serial_;
  char hexCar[3];  // should be static but I can't figure out how
};

// Implementations
#include "communication/fibre_protocol-impl.h"


template <uint16_t ENDPOINT>
void sendMsg(const uint16_t response_size,
             const odrive::endpoint_type_t<ENDPOINT>& payload, Print* serial) {
  sendMsgRaw(ENDPOINT | 0x8000, response_size,
             static_cast<const uint8_t*>(static_cast<const void*>(&payload)),
             sizeof(payload), serial);
}
template <uint16_t ENDPOINT, bool ACK>
void sendMsg(const odrive::endpoint_type_t<ENDPOINT>& payload, Print* serial) {
  sendMsgRaw(ENDPOINT | (ACK ? 0x8000 : 0), 0 /* response size */,
             static_cast<const uint8_t*>(static_cast<const void*>(&payload)),
             sizeof(payload), serial);
}

template <typename T>
void sendMsg(const uint16_t endpoint, const uint16_t response_size,
             const T& payload, Print* serial) {
  static_assert(!std::is_array<T>::value,
                "Please use sendMsgRaw for array types, since sizeof doesn't "
                "work properly for array types.");
  sendMsgRaw(endpoint, response_size,
             static_cast<const uint8_t*>(static_cast<const void*>(&payload)),
             sizeof(payload), serial);
}

uint8_t write_stream[1000];
void sendMsgRaw(const uint16_t endpoint, const uint16_t response_size,
                const uint8_t payload[], const size_t payload_size,
                Print* serial) {
  // TODO(gerry): make it clearer that we're directly writing to write_stream.
  uint8_t packet_size = fillPacket(seq_num, endpoint, response_size, payload,
                                   payload_size, &write_stream[3]);
  uint16_t stream_size =
      fillStream(&write_stream[3], packet_size, write_stream);
  serial->write(write_stream, stream_size);
}

uint8_t fillPacket(const uint16_t seq_num, const uint16_t endpoint,
                   const uint16_t response_size, const uint8_t payload[],
                   const size_t payload_size, uint8_t packet[]) {
  packet[0] = seq_num & 0xFF;
  packet[1] = seq_num >> 8;
  packet[2] = endpoint & 0xFF;
  packet[3] = endpoint >> 8;
  packet[4] = response_size & 0xFF;
  packet[5] = response_size >> 8;
  std::memcpy(&packet[6], payload, payload_size);
  if (endpoint & 0x7FFF) {
    packet[6 + payload_size] = JSON_CRC_16 & 0xFF;
    packet[7 + payload_size] = JSON_CRC_16 >> 8;
  } else {  // endpoint 0 (JSON)
    packet[6 + payload_size] = 0x01;
    packet[7 + payload_size] = 0x00;
  }
  return 8 + payload_size;
}

uint16_t fillStream(const uint8_t packet[], const uint8_t packet_size,
                    uint8_t stream[]) {
  stream[0] = SYNC;
  stream[1] = packet_size;
  stream[2] = crc8(stream, 2);
  // std::memcpy(&stream[3], packet, packet_size);
  uint16_t packet_crc = crc16(packet, packet_size);
  stream[3 + packet_size] =
      packet_crc >> 8;  // NOTE: this is big-endian for some reason
  stream[4 + packet_size] = packet_crc & 0xFF;
  return 5 + packet_size;
}

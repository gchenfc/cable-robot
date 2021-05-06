uint8_t requestInfo(uint16_t node, uint8_t cmdId, bool rtr=false);
uint8_t sendBytes(uint16_t node, uint8_t cmdId, uint8_t *data, bool rtr=false);
uint8_t sendInt32(uint16_t node, uint8_t cmdId, int32_t d1, int32_t d2 = 0);
uint8_t sendFloat(uint16_t node, uint8_t cmdId, float d1, float d2 = 0);
uint8_t sendInputPos(uint16_t node, uint8_t cmdId, float pos, int16_t vel, int16_t torque);

uint8_t requestInfo(uint16_t node, uint8_t cmdId, bool rtr) {
  uint8_t data[8];
  memset(data, 0, 8);
  return sendBytes(node, cmdId, data, rtr);
}

void read_float(uint8_t *data, float *float1, float *float2) {
  memcpy(float1, data, 4);
  memcpy(float2, data+4, 4);
}

void read_int32(uint8_t *data, int32_t *i1, int32_t *i2) {
  memcpy(i1, data, 4);
  memcpy(i2, data+4, 4);
}

uint8_t sendInt32(uint16_t node, uint8_t cmdId, int32_t d1, int32_t d2) {
  uint8_t data[8];
  memcpy(&data, (uint8_t*) (&d1), 4);
  memcpy(((uint8_t *)&data)+4, (uint8_t*) (&d2), 4);
  return sendBytes(node, cmdId, data);
}

uint8_t sendFloat(uint16_t node, uint8_t cmdId, float d1, float d2) {
  uint8_t data[8];
  if (isnan(d1) || isnan(d2)) {
    SerialD.print("Tried to send nan: ");
    SerialD.print(node);
    SerialD.print(" - ");
    SerialD.println(cmdId);
    return 0;
  }
  memcpy(&data, (uint8_t*) (&d1), 4);
  memcpy(((uint8_t *)&data)+4, (uint8_t*) (&d2), 4);
  if (cmdId == 0x0E)
    tset[node] = d1;
  return sendBytes(node, cmdId, data);
}

uint8_t sendInputPos(uint16_t node, uint8_t cmdId, float pos, int16_t vel, int16_t torque) {
  uint8_t data[8];
  memcpy(&data, (uint8_t*) (&pos), 4);
  memcpy(((uint8_t *)&data)+4, (uint8_t*) (&vel), 2);
  memcpy(((uint8_t *)&data)+6, (uint8_t*) (&torque), 2);
  return sendBytes(node, cmdId, data);
}

uint8_t sendBytes(uint16_t node, uint8_t cmdId, uint8_t *data, bool rtr) {
  CAN_message_t outMsg;
  outMsg.id = cmdId | (node << 5);
  outMsg.flags.remote = rtr;
  outMsg.len = 8;
  outMsg.flags.extended = 0;
  memcpy(outMsg.buf, data, 8);
  uint8_t success = Can0.write(outMsg);
  return success;
}

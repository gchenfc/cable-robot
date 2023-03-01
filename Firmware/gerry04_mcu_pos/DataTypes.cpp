#include "DataTypes.h"
#include "OdriveComm.h"

void Msg_t::toCan(CAN_message_t &outMsg) {
  outMsg.id = (node << 5) | (cmd & 0b11111);
  outMsg.rtr = OdriveComm::isRtr(cmd);
  outMsg.len = 8;
  outMsg.ext = 0;
  memcpy(outMsg.buf, buf, 8);
}

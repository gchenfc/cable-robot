#include "SerialComm.h"
#include "OdriveComm.h"
#include "Controller.h"
#include <FlexCAN.h>

bool checksum(CharBuffer_t &buf) {
  uint8_t sum = 0;
  for (int i = 0; i < (buf.size-2); i++) {
    sum += buf.buffer[i];
  }
  return sum == buf.buffer[buf.size-2];
}

void SerialComm::read() {
  while (serial_.available()) {
    inBuf_.append(serial_.read());
    if (inBuf_.isDone()) {
      if (!inBuf_.checksumIsGood()) {
        Msg_t faultMsg;
        faultMsg.node = 8;
        faultMsg.cmd = 31;
        write(faultMsg);
      } else {
        Msg_t msg;
        inBuf_.fillMsg(msg);
        if (msg.node < 4) {
          msgBuf_.push(msg);
        } else if (msg.node >= 4 && msg.node < 8) {
          axis_[msg.node-4].readSerMsg(msg, *this);
        } else if (msg.node == 8) {
          // TODO(gerry): process
          if (msg.cmd == 1) {
            odrv_->addToBuffer(msgBuf_);
          }
          controller_->processMsg(msg, *this);
        }
        // // ack
        // msg.node += 10;
        // write(msg);
      }

      inBuf_.clear();
    }
  }
}

void SerialComm::write(const Msg_t &msg) {
  outBuf_.fillMe(msg);
  serial_.write(outBuf_.buffer, outBuf_.size);
}

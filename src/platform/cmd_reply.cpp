#include "cmd_reply.h"

// Packet end marker, shared with main.cpp's binary protocol.
#define PKT_END 0x55

CommandReply Reply;

void CommandReply::beginBridgeCapture() {
  len_ = 0;
  bridge_ = true;
}

void CommandReply::endBridgeCapture() {
  flushFrame();  // emit any trailing line not terminated by '\n'
  bridge_ = false;
}

// Emit the buffered line as one reply frame, then reset. Empty lines are
// dropped so blank println()s don't put zero-length frames on the wire.
void CommandReply::flushFrame() {
  if (len_ == 0) {
    return;
  }
  Serial1.write((uint8_t)REPLY_START);
  Serial1.write((uint8_t)len_);
  Serial1.write(buf_, len_);
  Serial1.write((uint8_t)PKT_END);
  len_ = 0;
}

size_t CommandReply::write(uint8_t c) {
  Serial.write(c);  // mirror to the USB-CDC CLI always
  if (!bridge_) {
    return 1;
  }
  if (c == '\n') {
    flushFrame();
  } else if (c != '\r') {
    if (len_ >= kMaxFramePayload) {
      flushFrame();  // split a line longer than one frame
    }
    buf_[len_++] = c;
  }
  return 1;
}

size_t CommandReply::write(const uint8_t* buffer, size_t size) {
  for (size_t i = 0; i < size; ++i) {
    write(buffer[i]);
  }
  return size;
}

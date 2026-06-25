// CLI reply tee. The CLI dispatcher (serial_commands / calibration / memory)
// prints to this object instead of Serial directly. It always mirrors output to
// the USB-CDC Serial (unchanged bench behaviour), and -- while bridge capture is
// active -- also frames each completed line onto Serial1 as a reply packet so
// the PC app can show responses to commands that arrived over the bridge.
//
// Reply frame (Platform -> PC, on Serial1): [0xAC][len u8][len ASCII bytes][0x55]
// Distinct START byte from telemetry (0xAA) and command (0xAB) frames so the PC
// parser can demux all three from the one stream. '\r' is dropped; each '\n'
// flushes a frame; empty lines emit nothing; overlong lines split across frames.

#ifndef CMD_REPLY_H
#define CMD_REPLY_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#define REPLY_START 0xAC  // reply frame marker (Platform -> PC on Serial1)

class CommandReply : public Print {
 public:
  // Bracket a bridge-sourced command so its output is also framed onto Serial1.
  // USB-sourced commands leave capture off and only reach the USB Serial.
  void beginBridgeCapture();
  void endBridgeCapture();

  // Print interface. Mirrors to USB Serial always; frames to Serial1 when
  // bridge capture is active.
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buffer, size_t size) override;

 private:
  void flushFrame();

  static constexpr size_t kMaxFramePayload = 250;  // len is a u8
  uint8_t buf_[kMaxFramePayload];
  size_t len_ = 0;
  bool bridge_ = false;
};

extern CommandReply Reply;

#endif

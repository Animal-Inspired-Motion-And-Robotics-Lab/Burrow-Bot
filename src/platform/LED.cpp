// Thin wrapper around digitalWrite() so the rest of the firmware doesn't have
// to remember the XIAO ESP32-S3's active-low onboard LED polarity. Set the
// polarity once in ledInit(); on/off/flash do the right thing afterwards.

#include "LED.h"

#include <Arduino.h>

namespace {

static int gLedPin = LED_BUILTIN;
static bool gActiveHigh = true;

// Convert the logical on/off state to the right HIGH/LOW level for the
// configured polarity. On the XIAO ESP32-S3 the user LED is active-low, so
// main.cpp passes active_high = false at boot.
inline uint8_t ledOnLevel(void) {
  return gActiveHigh ? HIGH : LOW;
}

inline uint8_t ledOffLevel(void) {
  return gActiveHigh ? LOW : HIGH;
}

}  // namespace

void ledInit(int pin, bool active_high) {
  gLedPin = pin;
  gActiveHigh = active_high;

  pinMode(gLedPin, OUTPUT);
  digitalWrite(gLedPin, ledOffLevel());
}

void ledOn(void) {
  pinMode(gLedPin, OUTPUT);
  digitalWrite(gLedPin, ledOnLevel());
}

void ledOff(void) {
  pinMode(gLedPin, OUTPUT);
  digitalWrite(gLedPin, ledOffLevel());
}

// Blocking blink helper. delay() runs `count` on/off cycles with equal on and
// off times. Used at boot, during calibration, and on each detection — all
// places where blocking briefly is OK.
void ledFlash(uint8_t count, uint32_t ms_per_flash) {
  for (uint8_t i = 0; i < count; ++i) {
    ledOn();
    delay(ms_per_flash);
    ledOff();
    delay(ms_per_flash);
  }
}

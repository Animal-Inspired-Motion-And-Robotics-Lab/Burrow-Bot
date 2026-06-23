// Tiny LED helper. Hides the active-low/active-high polarity behind a single
// init call so the rest of the firmware can just say ledOn() / ledFlash().

#ifndef LED_H
#define LED_H

#include <stdbool.h>
#include <stdint.h>

// Configure pin and polarity. active_high=true means HIGH lights the LED;
// pass false for the XIAO ESP32-S3's active-low onboard LED.
void ledInit(int pin, bool active_high);

// Toggle the configured pin using the configured polarity.
void ledOn(void);
void ledOff(void);

// Blocking blink: `count` cycles of on/off, each phase `ms_per_flash` long.
void ledFlash(uint8_t count, uint32_t ms_per_flash);

#endif

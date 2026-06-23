// Firmware entry point. Orchestrates the per-tick pipeline:
//
//   ldc1101_read()      raw (Rp, L)
//        ↓
//   appendMeasurement() filter + rotate + store
//        ↓
//   crackDetectionCheck() shape + planarity + dedup
//        ↓
//   telemetryEmitSample() emit Teleplot stream + optional crack/debug fields
//
// All runtime tunables live in serial_commands.cpp; we read snapshots of its
// config + state each tick and pass them to the relevant module.

#include <Arduino.h>
#include "LED.h"
#include "crack_detection.h"
#include "ldc1101.h"
#include "measurement_arrays.h"
#include "serial_commands.h"
#include "telemetry.h"

const char* fw_version = "0.2.8";

// Boot defaults — all of these are reconfigurable at runtime via the CLI.
// Listed in the order setup() consumes them.

// Sample period for the main loop. The CLI's `delay <ms>` command overwrites
// the live copy of this in serial_commands' state.
static constexpr uint32_t kDefaultReadingDelayMs = 25;

// LC tank parameters used to drive RP_SET / TC1 / TC2 / DIG_CONF register
// derivation inside the driver. Update both the value and the matching
// stacked-inductor variant comment together.
// For the stacked inductors, L = 11.8, 42.6, 90.0 uH.
static constexpr float kSensorL_H = 42.6e-6f; // uH = 1e-6H
static constexpr float kSensorC_F = 220e-12f;  // pF = 1e-12F

// For the stacked inductors, modeled Q values are 23.6, 24.6, 25.6 (220 pF).
static constexpr float kSensorQ = 25.0f;

// External mux/switch driven by the driver before configuring. Unused on the
// current wiring (-1 GPIO disables it).
static constexpr int kSwitchEnable = 0;
static constexpr int kSwitchGpio = -1;

// Onboard LED. The XIAO ESP32-S3 user LED is active-low.
static constexpr int kLedPin = LED_BUILTIN;
static constexpr bool kLedActiveHigh = false;

// Timestamp of the last emitted tick — used by the loop to enforce
// reading_delay_ms without blocking on delay().
static uint32_t lastPrintMs = 0;

void setup() {
  // Note: USB-CDC on the XIAO ESP32-S3 ignores the firmware-side baud, so the
  // number passed to Serial.begin() doesn't have to match platformio.ini's
  // monitor speed. Keeping the call here mostly for portability.
  Serial.begin(9600);
  ledInit(kLedPin, kLedActiveHigh);
  ledFlash(10, 150);   // Quick blink so the operator sees the chip restart.
  delay(5000);         // Give the host time to re-enumerate the USB-CDC.
  Serial.print("LDC Testing, FW Version: "); Serial.println(fw_version);

  // Seed the CLI with our boot defaults. After this, every CLI command
  // mutates this in-place via the gConfig / gState globals in serial_commands.
  serial_command_config_t commandConfig = {kSensorL_H, kSensorC_F, kSensorQ,
      kSwitchEnable, kSwitchGpio};
  serial_command_state_t initialState = {LDC1101_MODE_RP_L, LDC_SPEED_BALANCED_1,
      true, false, true, false, kDefaultReadingDelayMs};

  // ldc1101_init() prepares the SPI bus; serialCommandsInit() then pushes the
  // seeded sensor/mode/speed values to the chip via ldc1101_configure() so
  // we don't have to call configure() ourselves here.
  ldc1101_init();
  serialCommandsInit(&commandConfig, &initialState);

  // Moving-average smoothing window applied before rotation and storage.
  // Set to 1 to disable filtering entirely.
  setFilterWindow(25);

  // Crack detector tuning. These mirror the `crack_*` serial commands and can
  // be retuned live; the values here are just the boot defaults.
  crack_detection_config_t crackConfig = {
      0.01f,    // threshold             — min fitted peak height above rotated baseline
      110,      // window_samples        — fit window (raise as robot speed drops)
      0.5f,     // min_parabola_r2       — fit must explain at least this much variance
      1.0f,     // max_deviation_rad     — ≈57°; arctan(|m|) where m is Ω/sample
                //                         slope of Rp linear fit. Caps the tilt
                //                         of the (t,Rp,L) curve off the t-L
                //                         plane, rejecting Rp drift from
                //                         material transitions while still
                //                         tolerating sensor-noise wiggle.
      220.0f    // length_estimate_scale — thou per µH (peak × scale = crack_size)
  };
  crackDetectionInit(&crackConfig);

  Serial.println("LDC1101 initialized");
}

void loop() {
  // Drain any queued CLI input first so user changes apply *this* tick.
  serialCommandsPoll();
  serial_command_state_t state = serialCommandsGetState();
  serial_command_config_t config = serialCommandsGetConfig();

  // `stream off` halts emission without disturbing CLI responsiveness. A tiny
  // delay yields to other tasks instead of hot-spinning.
  if (!state.streaming_enabled) {delay(2); return;}

  // Non-blocking pacing: only do a real read+emit when enough wall time has
  // elapsed since the last one. millis() wraparound is fine because the
  // subtraction is unsigned.
  uint32_t now = millis();
  if (now - lastPrintMs >= state.reading_delay_ms) {
    lastPrintMs = now;

    // 1. Read raw (Rp, L) from the LDC1101.
    ldc1101_measurement_t m = ldc1101_read(config.sensor_c_f);

    // 2. Push through smoothing + rotation, store in history.
    appendMeasurement(m.Rp_ohms, m.L_uH);

    // 3. Run the full detector chain on the newly extended window.
    crack_detection_result_t crackResult = {};
    bool crackDetected = crackDetectionCheck(&crackResult);

    // 4. Only flash for a crack once the user has calibrated the rotation —
    //    pre-calibration detections aren't trustworthy.
    if (state.rotated && crackDetected) {
      ledFlash(3, 20);
    }

    // 5. Emit the per-tick Teleplot line (+ optional crack/debug fields).
    telemetryEmitSample(now, &state, crackDetected, &crackResult);
  }
}
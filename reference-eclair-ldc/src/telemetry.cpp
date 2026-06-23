// Streaming serial output for the measurement pipeline.
//
// This module is intentionally the only place that writes the per-tick
// ">Rp:.../>L:..." line, so the wire format lives in exactly one file. The
// output is Teleplot-flavored: each ">name:value" is one Teleplot datapoint,
// and the "|xy" suffix tells Teleplot to render Rp vs L as an X-Y plot.
//
// Three categories of output per tick:
//   1. Always: the >Rp / >L / >t stream line (one per tick).
//   2. On a confirmed detection with crack_output on: >mag/>crack_x/
//      >crack_size/>width fields appended to the same line.
//   3. On a rejection with crack_debug on: a separate >reason line, plus a
//      plain-text key=value debug line with the live tuning knobs.

#include "telemetry.h"

#include <Arduino.h>

#include "ldc1101.h"
#include "measurement_arrays.h"

// Build and emit one tick's serial output.
//   now_ms          - timestamp from main.cpp's millis()
//   state           - current CLI state (controls which extras are emitted)
//   crack_detected  - return value of crackDetectionCheck()
//   crack           - result struct (or nullptr) from crackDetectionCheck()
void telemetryEmitSample(uint32_t now_ms,
                         const serial_command_state_t* state,
                         bool crack_detected,
                         const crack_detection_result_t* crack) {
  if (state == nullptr) return;

  // Show calibrated/rotated values once the user has run "calibrate" and
  // turned rotation on; otherwise show the raw filtered values. In LHR mode
  // the LDC1101 does not produce a meaningful Rp, so report 0.
  float rpOut = state->rotated ? getLatestRotatedRp() : getLatestFilteredRp();
  float lOut  = state->rotated ? getLatestRotatedL()  : getLatestFilteredL();
  if (state->mode == LDC1101_MODE_LHR) rpOut = 0.0f;

  Serial.print(">Rp:"); Serial.print(rpOut, 3);
  Serial.print(">L:");  Serial.print(lOut, 6);

  if (state->crack_output && crack_detected && crack != nullptr) {
    // Translate the fit's vertex position (in samples, 0 = oldest in window)
    // into an absolute timestamp by stepping back from `now_ms`:
    //   crackXMs = now - (samples_after_vertex * sample_period)
    // and clamp to 0 in case the math underflows on a tiny millis() counter.
    const float windowSamples = static_cast<float>(crackDetectionGetWindowSamples());
    const float sampleDtMs = static_cast<float>(state->reading_delay_ms);
    const float scaledCrackSize =
        crack->fit_peak_height * crackDetectionGetLengthEstimateScale();
    float crackXMs = static_cast<float>(now_ms) -
                     ((windowSamples - 1.0f - crack->fit_peak_x_samples) * sampleDtMs);
    if (crackXMs < 0.0f) {
      crackXMs = 0.0f;
    }

    Serial.print(">mag:");        Serial.print(crack->fit_peak_height, 6);
    Serial.print(">crack_x:");    Serial.print(crackXMs, 3);
    Serial.print(">crack_size:"); Serial.print(scaledCrackSize, 6);
    Serial.print(">width:");      Serial.print(crack->fit_width_samples, 6);
  }
  Serial.print(">t:"); Serial.print(now_ms);
  Serial.println("|xy");  // Teleplot directive: render Rp vs L as X-Y.

  if (state->crack_debug_output && crack != nullptr && crack->reject_reason != nullptr) {
    // Why this tick's parabola was rejected. Emitted as its own Teleplot
    // text-log line so it shows up alongside the >Rp/>L stream.
    Serial.print(">reason:");
    Serial.println(crack->reject_reason);
  }

  if (state->crack_debug_output && crack != nullptr) {
    // Per-tick crack-detector state. Tuning knob labels match the `status`
    // command and the `crack_*` set commands so a debug line is self-describing.
    Serial.print("detected=");          Serial.print(crack->detected ? 1 : 0);
    Serial.print(" reject_reason=");    Serial.print(crack->reject_reason ? crack->reject_reason : "-");
    Serial.print(" fit_peak=");         Serial.print(crack->fit_peak_height, 2);
    Serial.print(" fit_width=");        Serial.print(crack->fit_width_samples, 2);
    Serial.print(" fit_r2=");           Serial.println(crack->fit_r2, 2);
  }
}

// Multi-stage crack detector built on top of the rotated measurement stream.
// See crack_detection.cpp for the full algorithm description and pipeline.

#ifndef CRACK_DETECTION_H
#define CRACK_DETECTION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Tuning knobs. All fields are runtime-mutable via the serial CLI.
typedef struct {
  float threshold;              // Minimum fitted parabola peak height (above baseline).
  size_t window_samples;        // Rolling window length used for fit + deviation.
  float min_parabola_r2;        // Minimum R² of the parabola fit (0..1).
  float max_deviation_rad;      // Max angular deviation of the 3D (t,Rp,L) curve
                                // from the t-L plane, in [0, π/2]. Computed as
                                // arctan(|m|) where m is the least-squares slope
                                // of rotated Rp vs sample index — geometrically
                                // the tilt of the (parabolic L + linear Rp)
                                // curve's plane away from the t-L plane. 0 = the
                                // curve must lie exactly in the t-L plane (pure
                                // L bump, no Rp drift); π/2 disables the check.
  float length_estimate_scale;  // Scales fit peak height into a length estimate.
} crack_detection_config_t;

// Per-tick result published by crackDetectionCheck(). Fit values are populated
// whenever a parabola fit was possible — even on rejection — so the debug
// stream can show what the fit looked like. On detection, reject_reason is
// nullptr; on rejection it points to a static reason string.
typedef struct {
  bool detected;
  float fit_peak_height;       // Vertex height above the rotated baseline.
  float fit_peak_x_samples;    // Vertex location, 0..window_samples-1 (oldest→newest).
  float fit_half_peak_height;  // = fit_peak_height / 2 (kept for debug).
  float fit_width_samples;     // Full width where the parabola crosses zero.
  float fit_r2;                // Coefficient of determination of the fit.
  // When `detected` is false and the parabola fit succeeded, this points to a
  // static string explaining which check rejected the window (e.g. "low_r2",
  // "threshold", "deviation", "refractory", "held").
  // nullptr means no rejection reason (either detected, or no fit attempted).
  const char* reject_reason;
} crack_detection_result_t;

// Adopt `config` (or, if null, just re-clamp the current values) and reset the
// dedup / refractory state machine.
void crackDetectionInit(const crack_detection_config_t* config);

// Per-tick check. Returns true exactly when a new detection fires (not on a
// "held" continuation of the same event). Always populates `result` if given.
bool crackDetectionCheck(crack_detection_result_t* result);

// Runtime tuning. Each set() clamps to the valid range; each get() returns the
// post-clamp value currently in use. All paired with a `crack_*` serial command.
void crackDetectionSetWindowSamples(size_t window_samples);
size_t crackDetectionGetWindowSamples(void);

void crackDetectionSetThreshold(float threshold);
float crackDetectionGetThreshold(void);

void crackDetectionSetMinParabolaR2(float min_parabola_r2);
float crackDetectionGetMinParabolaR2(void);

// Max allowed angular deviation of the 3D (t, Rp, L) curve from the t-L plane.
// Computed as arctan(|m|) where m is the slope (Ω per sample) of the
// least-squares linear fit of rotated Rp vs sample index over the window.
// Geometrically: for a parabolic L plus linear Rp, the curve lies in a plane
// tilted by exactly arctan(|m|) from the t-L plane, so this caps that tilt.
// Clamped to [0, π/2]; π/2 disables the check.
void crackDetectionSetMaxDeviationRad(float max_deviation_rad);
float crackDetectionGetMaxDeviationRad(void);

void crackDetectionSetLengthEstimateScale(float length_estimate_scale);
float crackDetectionGetLengthEstimateScale(void);

#endif

// Multi-stage crack detector.
//
// Stage 1 (shape): least-squares fit a parabola to the most recent
//   `crack_window` rotated-L samples. Accept the window if the fit explains the
//   data (R² >= crack_r2) and the fitted peak is tall enough
//   (peak >= crack_threshold).
//
// Stage 2 (direction): characterize the curve's orientation in 3D (t, Rp, L)
//   space and reject when it tilts too far away from the t-L plane. A real
//   crack signature should lie mostly in the t-L plane — as the probe sweeps
//   in time, L bumps while Rp stays at the calibrated substrate baseline. If
//   Rp drifts across the window the curve tilts off the t-L plane toward the
//   L-Rp plane, which is the signature of substrate motion (material patch
//   boundary, baseline slide, lift-off).
//
//   We characterize the tilt analytically: for a parabolic L plus a linear
//   Rp(t) = m·t + b, the 3D curve lies exactly in the plane spanned by
//   (1, m, 0) and (0, 0, 1), whose unit normal is (−m, 1, 0)/√(m²+1). The
//   angle between that plane and the t-L plane (normal (0, 1, 0)) is
//   arctan(|m|), so the tilt is captured entirely by the slope m of rotated
//   Rp vs sample index. We fit m by least squares and reject when
//   arctan(|m|) > crack_deviation.
//
// On a confirmed detection the detector arms a refractory cooldown so that the
// same physical crack does not emit multiple times as its tail slides through
// the window; `gPreviousQualified` adds a second dedup layer (see notes inside
// crackDetectionCheck).

#include "crack_detection.h"

#include <math.h>

#include "measurement_arrays.h"

namespace {

static constexpr float kPi = 3.14159265358979323846f;
// Numerical floor used for "matrix singular" / "peak essentially zero" tests
// inside the parabola fit. Anything below this is treated as not-a-real-value.
static constexpr float kPeakEpsilon = 1.0e-6f;
static constexpr float kDefaultMinParabolaR2 = 0.90f;
// Default deviation cap ~57°. The implicit unit is arctan(Ω per sample): at
// this threshold and the default 110-sample window, the curve's linear-fit Rp
// drift may reach tan(1.0) · 109 ≈ 170 Ω across the window before rejection.
// Loose enough that sensor-noise wiggle on a calibrated substrate never trips
// it, tight enough to reject the multi-kΩ drift a material transition produces.
static constexpr float kDefaultMaxDeviationRad = 1.0f;

// Live tuning state. Populated by crackDetectionInit() and the
// crackDetectionSet*() commands; read on every tick by crackDetectionCheck().
// Defaults here are only used if init() is somehow skipped — main.cpp passes
// the real config at boot.
static crack_detection_config_t gConfig = {0.1f, 100, kDefaultMinParabolaR2,
                                           kDefaultMaxDeviationRad, 1.0f};
static bool gInitialized = false;

// Cross-tick state for the two dedup mechanisms:
//   - gPreviousQualified: true if the prior tick passed shape+deviation. While
//     this is true, even an otherwise-good window will not re-fire (prevents a
//     single crack from emitting once per tick as it slides through).
//   - gRefractoryRemaining: cooldown counter loaded after a detection. Counts
//     down by 1 every tick regardless of qualification, so it's a wall-clock
//     timer in units of sample ticks.
static bool gPreviousQualified = false;
static size_t gRefractoryRemaining = 0;

// Decide how long the cooldown after a detection should be, in sample ticks.
// We want at least the fit's reported width (so the same crack's tail can roll
// out of the window) but never shorter than a quarter of the configured window
// (so a wide window doesn't fire on every quarter-window slide).
size_t computeRefractorySamples(float fitWidthSamples) {
  size_t widthBased = 1;
  if (isfinite(fitWidthSamples) && fitWidthSamples > 0.0f) {
    widthBased = static_cast<size_t>(ceilf(fitWidthSamples));
    if (widthBased < 1) {
      widthBased = 1;
    }
  }

  size_t windowBased = gConfig.window_samples / 4;
  if (windowBased < 1) {
    windowBased = 1;
  }

  return (widthBased > windowBased) ? widthBased : windowBased;
}

// Stage 2 helper: angular deviation of the 3D (t, Rp, L) curve from the t-L
// plane. For a parabolic L plus a linear-in-time Rp drift with slope m, the
// curve lies exactly in the plane spanned by (1, m, 0) and (0, 0, 1), whose
// unit normal is (−m, 1, 0)/√(m²+1). The angle that plane makes with the t-L
// plane (normal (0, 1, 0)) is arctan(|m|) — equivalently, the tangent at the
// parabola vertex is (1, m, 0), and its angle off the t-L plane is also
// arctan(|m|). So the curve's "direction of change" is captured entirely by
// the least-squares slope m of rotated Rp vs sample index.
//
// Returns false if the window can't be read.
bool getDeviationAngleForWindow(size_t sampleCount, float* deviationRad) {
  if (deviationRad == nullptr || sampleCount < 2) {
    return false;
  }

  // Accumulate Σi, Σi², ΣRp, ΣiRp for the closed-form linear fit. We don't
  // need ΣRp² (no R² for this fit).
  float sumI = 0.0f;
  float sumI2 = 0.0f;
  float sumRp = 0.0f;
  float sumIRp = 0.0f;
  for (size_t i = 0; i < sampleCount; ++i) {
    float rp = 0.0f;
    float l = 0.0f;
    const size_t samplesAgo = sampleCount - 1 - i;
    if (!getRecentRotatedSample(samplesAgo, &rp, &l)) {
      return false;
    }
    const float fi = static_cast<float>(i);
    sumI += fi;
    sumI2 += fi * fi;
    sumRp += rp;
    sumIRp += fi * rp;
  }

  // Slope m = (N·ΣiRp − Σi·ΣRp) / (N·Σi² − (Σi)²). Denominator is the
  // variance of i scaled by N — strictly positive for N ≥ 2, but guard
  // anyway against float underflow on a one-sample edge case.
  const float n = static_cast<float>(sampleCount);
  const float denom = n * sumI2 - sumI * sumI;
  if (denom < kPeakEpsilon) {
    return false;
  }
  const float slope = (n * sumIRp - sumI * sumRp) / denom;

  *deviationRad = atanf(fabsf(slope));
  return true;
}

// Closed-form least-squares fit of y = a·x² + b·x + c to the most recent
// `sampleCount` rotated-L samples, with x = 0..sampleCount-1 (oldest to
// newest). y is taken relative to rotationCenterL so a no-crack baseline sits
// near zero.
//
// Outputs are derived analytically from a/b/c:
//   peakHeight      = c − b² / (4a)            (the vertex value)
//   peakXSamples    = −b / (2a)                (vertex x, samples-from-oldest)
//   halfPeakHeight  = peakHeight / 2           (kept for the debug stream)
//   widthSamples    = 2·√(−peakHeight / (2a))  (full width at zero crossing
//                                               relative to baseline)
//   fitR2           = standard coefficient of determination
//
// Returns false (without setting outputs) if the matrix is singular, the
// parabola opens upward (a ≥ 0), the vertex is outside the window, or the
// fitted peak/width is non-positive. Callers treat false as "no fit attempted"
// and emit no rejection reason.
bool fitParabolaForWindow(size_t sampleCount,
                          float* peakHeight,
                          float* peakXSamples,
                          float* halfPeakHeight,
                          float* widthSamples,
                          float* fitR2) {
  if (peakHeight == nullptr || peakXSamples == nullptr ||
      halfPeakHeight == nullptr ||
      widthSamples == nullptr || fitR2 == nullptr) {
    return false;
  }

  if (sampleCount < 3) {
    return false;
  }

  // y is measured relative to the calibrated rotation center so the fit
  // describes a peak above the substrate baseline rather than above zero.
  float rotationCenterRp = 0.0f;
  float rotationCenterL = 0.0f;
  getRotationCenter(&rotationCenterRp, &rotationCenterL);

  // Accumulate the moments needed for the normal equations. One pass over the
  // window suffices for all of Σx, Σx², Σx³, Σx⁴, Σy, Σxy, Σx²y.
  float sumX = 0.0f;
  float sumX2 = 0.0f;
  float sumX3 = 0.0f;
  float sumX4 = 0.0f;
  float sumY = 0.0f;
  float sumXY = 0.0f;
  float sumX2Y = 0.0f;

  for (size_t i = 0; i < sampleCount; ++i) {
    float rp = 0.0f;
    float l = 0.0f;
    const size_t samplesAgo = sampleCount - 1 - i;
    if (!getRecentRotatedSample(samplesAgo, &rp, &l)) {
      return false;
    }

    const float x = static_cast<float>(i);
    const float y = l - rotationCenterL;
    const float x2 = x * x;

    sumX += x;
    sumX2 += x2;
    sumX3 += x2 * x;
    sumX4 += x2 * x2;
    sumY += y;
    sumXY += x * y;
    sumX2Y += x2 * y;
  }

  const float n = static_cast<float>(sampleCount);

  // Normal equations  M · [a;b;c] = v  with
  //   M = [Σx⁴ Σx³ Σx²;  Σx³ Σx² Σx;  Σx² Σx n]
  //   v = [Σx²y;  Σxy;  Σy]
  float m00 = sumX4;
  float m01 = sumX3;
  float m02 = sumX2;
  float m10 = sumX3;
  float m11 = sumX2;
  float m12 = sumX;
  float m20 = sumX2;
  float m21 = sumX;
  float m22 = n;

  float v0 = sumX2Y;
  float v1 = sumXY;
  float v2 = sumY;

  // Forward-eliminate to upper-triangular, then back-substitute. Bail out at
  // any vanishing pivot — that's a degenerate window we can't fit.
  if (fabsf(m00) < kPeakEpsilon) {
    return false;
  }
  const float f10 = m10 / m00;
  const float f20 = m20 / m00;
  m10 -= f10 * m00;
  m11 -= f10 * m01;
  m12 -= f10 * m02;
  v1 -= f10 * v0;
  m20 -= f20 * m00;
  m21 -= f20 * m01;
  m22 -= f20 * m02;
  v2 -= f20 * v0;

  if (fabsf(m11) < kPeakEpsilon) {
    return false;
  }
  const float f21 = m21 / m11;
  m21 -= f21 * m11;
  m22 -= f21 * m12;
  v2 -= f21 * v1;

  if (fabsf(m22) < kPeakEpsilon) {
    return false;
  }

  // Back-substitute for the three coefficients of y = a·x² + b·x + c.
  const float c = v2 / m22;
  const float b = (v1 - m12 * c) / m11;
  const float a = (v0 - m01 * b - m02 * c) / m00;

  // For a crack we expect a downward-opening parabola (a < 0). Reject
  // upward-opening fits and any NaN/Inf — those don't describe a peak.
  if (!(a < -kPeakEpsilon) || !isfinite(a) || !isfinite(b) || !isfinite(c)) {
    return false;
  }

  // Vertex must live inside the window — extrapolated peaks aren't credible.
  const float xVertex = -b / (2.0f * a);
  if (!(xVertex >= 0.0f) || !(xVertex <= (n - 1.0f)) || !isfinite(xVertex)) {
    return false;
  }

  // Peak height = c − b²/(4a). Above the baseline = positive (we subtracted
  // rotationCenterL during accumulation).
  const float fittedPeak = c - ((b * b) / (4.0f * a));
  if (!(fittedPeak > 0.0f) || !isfinite(fittedPeak)) {
    return false;
  }

  // Distance from the vertex to where the parabola crosses y = 0:
  //   y(xVertex ± Δ) = 0  ⇒  Δ² = −fittedPeak / (2a)
  // Full width is 2·Δ in samples.
  const float halfHeight = 0.5f * fittedPeak;
  const float halfWidthSquared = -fittedPeak / (2.0f * a);
  if (!(halfWidthSquared > 0.0f) || !isfinite(halfWidthSquared)) {
    return false;
  }

  const float fittedWidthSamples = 2.0f * sqrtf(halfWidthSquared);
  if (!(fittedWidthSamples > 0.0f) || !isfinite(fittedWidthSamples)) {
    return false;
  }

  // Compute R² = 1 − SSE/SST in a second pass over the same window.
  float sst = 0.0f;
  float sse = 0.0f;
  const float meanY = sumY / n;
  for (size_t i = 0; i < sampleCount; ++i) {
    float rp = 0.0f;
    float l = 0.0f;
    const size_t samplesAgo = sampleCount - 1 - i;
    if (!getRecentRotatedSample(samplesAgo, &rp, &l)) {
      return false;
    }

    const float x = static_cast<float>(i);
    const float y = l - rotationCenterL;
    const float yHat = (a * x * x) + (b * x) + c;
    const float err = y - yHat;
    const float dy = y - meanY;
    sse += err * err;
    sst += dy * dy;
  }

  // If the data has no variance at all, R² is undefined by the usual formula.
  // Treat zero residuals as a perfect fit; anything else as a worst-case zero.
  float r2 = 0.0f;
  if (sst <= kPeakEpsilon) {
    r2 = (sse <= kPeakEpsilon) ? 1.0f : 0.0f;
  } else {
    r2 = 1.0f - (sse / sst);
  }
  if (!isfinite(r2)) {
    return false;
  }

  *peakHeight = fittedPeak;
  *peakXSamples = xVertex;
  *halfPeakHeight = halfHeight;
  *widthSamples = fittedWidthSamples;
  *fitR2 = r2;
  return true;
}

}  // namespace

// Adopt the caller's tuning, clamp each field to its valid range, and reset
// the per-tick state machine. Safe to call repeatedly. Passing nullptr leaves
// gConfig untouched but still re-clamps and resets — handy as a "lazy init"
// fallback when a setter is called before main.cpp's explicit init.
void crackDetectionInit(const crack_detection_config_t* config) {
  if (config != nullptr) {
    gConfig = *config;
  }

  if (gConfig.threshold < 0.0f) {
    gConfig.threshold = 0.0f;
  }

  if (gConfig.window_samples < 1) {
    gConfig.window_samples = 1;
  }

  if (gConfig.min_parabola_r2 < 0.0f) {
    gConfig.min_parabola_r2 = 0.0f;
  }
  if (gConfig.min_parabola_r2 > 1.0f) {
    gConfig.min_parabola_r2 = 1.0f;
  }

  // Deviation threshold lives in [0, π/2] — the codomain of arctan(|m|) for
  // any real slope m. 0 = curve must lie exactly in the t-L plane;
  // π/2 effectively disables the check.
  if (gConfig.max_deviation_rad < 0.0f) {
    gConfig.max_deviation_rad = 0.0f;
  }
  if (gConfig.max_deviation_rad > kPi * 0.5f) {
    gConfig.max_deviation_rad = kPi * 0.5f;
  }

  if (gConfig.length_estimate_scale < 0.0f) {
    gConfig.length_estimate_scale = 0.0f;
  }

  gPreviousQualified = false;
  gRefractoryRemaining = 0;
  gInitialized = true;
}

// Per-tick entry point. Run the full check chain on whatever rolling window is
// currently in measurement_arrays and decide if this tick should emit a crack.
// Returns true exactly when a brand-new detection fires; false otherwise.
// When `result` is provided it always receives the fit values (or zeros) and,
// on a rejection, the reason that disqualified the window.
bool crackDetectionCheck(crack_detection_result_t* result) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  // Decrement the refractory clock unconditionally — it's wall time, not
  // "ticks where we qualified."
  if (gRefractoryRemaining > 0) {
    --gRefractoryRemaining;
  }

  // Zero the result up front so callers see a well-defined "no detection"
  // state even if we bail out at the parabola fit.
  if (result != nullptr) {
    result->detected = false;
    result->fit_peak_height = 0.0f;
    result->fit_peak_x_samples = 0.0f;
    result->fit_half_peak_height = 0.0f;
    result->fit_width_samples = 0.0f;
    result->fit_r2 = 0.0f;
    result->reject_reason = nullptr;
  }

  // --- Stage 0: try to fit a parabola at all. ---
  float fitPeakHeight = 0.0f;
  float fitPeakXSamples = 0.0f;
  float fitHalfPeakHeight = 0.0f;
  float fitWidthSamples = 0.0f;
  float fitR2 = 0.0f;
  if (!fitParabolaForWindow(gConfig.window_samples,
                            &fitPeakHeight, &fitPeakXSamples,
                            &fitHalfPeakHeight,
                            &fitWidthSamples, &fitR2)) {
    // Not even a usable fit — too few samples, singular matrix, upward
    // parabola, etc. We treat this as "haven't started looking" and emit no
    // reject_reason; reporting one every tick before warmup completes would
    // drown the debug stream.
    gPreviousQualified = false;
    return false;
  }

  // Fit succeeded — publish the numbers regardless of qualification so the
  // crack_debug stream can show what the fit looked like even on rejection.
  if (result != nullptr) {
    result->fit_peak_height = fitPeakHeight;
    result->fit_peak_x_samples = fitPeakXSamples;
    result->fit_half_peak_height = fitHalfPeakHeight;
    result->fit_width_samples = fitWidthSamples;
    result->fit_r2 = fitR2;
  }

  // --- Stage 1: shape check (R² then threshold, first failure wins). ---
  if (fitR2 < gConfig.min_parabola_r2) {
    if (result != nullptr) result->reject_reason = "low_r2";
    gPreviousQualified = false;
    return false;
  }
  if (fitPeakHeight < gConfig.threshold) {
    if (result != nullptr) result->reject_reason = "threshold";
    gPreviousQualified = false;
    return false;
  }

  // --- Stage 2: 3D direction check. ---
  // Compute the angular deviation of the curve from the t-L plane (= arctan
  // of the linear-fit slope of rotated Rp vs sample index) and reject when
  // it exceeds crack_deviation. If the helper can't produce an angle
  // (degenerate window after Stage 1 — essentially unreachable in practice)
  // we let the candidate through.
  float deviationRad = NAN;
  if (getDeviationAngleForWindow(gConfig.window_samples, &deviationRad) &&
      deviationRad > gConfig.max_deviation_rad) {
    if (result != nullptr) result->reject_reason = "deviation";
    gPreviousQualified = false;
    return false;
  }

  // --- Stage 3: dedup. ---
  // The window has passed shape + direction. Two reasons we still might not fire:
  //   - We're inside the cooldown that was loaded by an earlier detection
  //     (same physical crack still in the window, or back-to-back cracks).
  //   - The previous tick was also qualified, so this is the same crack
  //     emitting in two adjacent ticks — only the first should fire.
  if (gRefractoryRemaining > 0) {
    if (result != nullptr) result->reject_reason = "refractory";
    gPreviousQualified = true;
    return false;
  }
  if (gPreviousQualified) {
    if (result != nullptr) result->reject_reason = "held";
    return false;
  }

  // --- Detection. ---
  // Latch the dedup flags and arm a cooldown sized to the fit so the next
  // detection has to be a clearly separate event.
  gPreviousQualified = true;
  gRefractoryRemaining = computeRefractorySamples(fitWidthSamples);

  if (result != nullptr) {
    result->detected = true;
    result->fit_peak_height = fitPeakHeight;
    result->fit_peak_x_samples = fitPeakXSamples;
    result->fit_half_peak_height = fitHalfPeakHeight;
    result->fit_width_samples = fitWidthSamples;
    result->fit_r2 = fitR2;
  }

  return true;
}

// -----------------------------------------------------------------------------
// Runtime setters/getters for each tuning knob. Each one lazy-inits with
// nullptr (which preserves whatever was last configured but re-clamps) so
// callers can use them before main.cpp's explicit init has run. The same
// clamping rules from crackDetectionInit() are applied on every write.
// -----------------------------------------------------------------------------

void crackDetectionSetWindowSamples(size_t window_samples) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (window_samples < 1) {
    window_samples = 1;
  }
  gConfig.window_samples = window_samples;
}

size_t crackDetectionGetWindowSamples(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.window_samples;
}

void crackDetectionSetThreshold(float threshold) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (threshold < 0.0f) {
    threshold = 0.0f;
  }
  gConfig.threshold = threshold;
}

float crackDetectionGetThreshold(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.threshold;
}

void crackDetectionSetMinParabolaR2(float min_parabola_r2) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (min_parabola_r2 < 0.0f) {
    min_parabola_r2 = 0.0f;
  }
  if (min_parabola_r2 > 1.0f) {
    min_parabola_r2 = 1.0f;
  }
  gConfig.min_parabola_r2 = min_parabola_r2;
}

float crackDetectionGetMinParabolaR2(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.min_parabola_r2;
}

void crackDetectionSetMaxDeviationRad(float max_deviation_rad) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (max_deviation_rad < 0.0f) {
    max_deviation_rad = 0.0f;
  }
  if (max_deviation_rad > kPi * 0.5f) {
    max_deviation_rad = kPi * 0.5f;
  }
  gConfig.max_deviation_rad = max_deviation_rad;
}

float crackDetectionGetMaxDeviationRad(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.max_deviation_rad;
}

void crackDetectionSetLengthEstimateScale(float length_estimate_scale) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (length_estimate_scale < 0.0f) {
    length_estimate_scale = 0.0f;
  }
  gConfig.length_estimate_scale = length_estimate_scale;
}

float crackDetectionGetLengthEstimateScale(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.length_estimate_scale;
}

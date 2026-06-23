// Storage and preprocessing for the (Rp, L) sample stream.
//
// Every tick from main.cpp calls appendMeasurement(), which does three things:
//   1. Pushes the raw sample through a moving-average filter (smoothing).
//   2. Optionally rotates the filtered sample about a calibrated center so
//      that the substrate's baseline trend lies along the x-axis. Crack
//      excursions then live on the rotated L axis.
//   3. Stores both the filtered and rotated samples in a fixed-length history
//      so downstream consumers (crack_detection, calibration) can look back
//      over the most recent N samples.

#include "measurement_arrays.h"

#include <math.h>

// Length of the rolling history kept for downstream lookback. 1000 samples is
// 25 s at the default 25 ms tick — plenty of room for the largest crack window
// or PCA window the CLI lets you ask for.
static constexpr size_t array_length = 1000;

// Filtered (post-smoothing) history of the raw input.
static float Rp_array[array_length] = {0.0f};
static float L_array[array_length] = {0.0f};
// Same indices but after the per-sample rotation step. When rotation is off
// these mirror the filtered arrays.
static float rotatedRpArray[array_length] = {0.0f};
static float rotatedLArray[array_length] = {0.0f};
static size_t measurementCount = 0;  // number of valid entries in the histories

// Moving-average ring buffer used by filterSample(). Sized to the largest
// window the CLI allows; the active window is `filterWindow`.
static constexpr size_t kMaxFilterWindow = 512;
static float rpRing[kMaxFilterWindow] = {0.0f};
static float lRing[kMaxFilterWindow] = {0.0f};
static size_t ringHead = 0;     // next write index
static size_t ringFilled = 0;   // valid entries, capped at kMaxFilterWindow
static size_t filterWindow = 25; // 1 = pass-through

// Most-recent filtered and rotated outputs. Telemetry reads these directly so
// it doesn't have to dig into the history arrays just to print one sample.
static float latestFilteredRp = 0.0f;
static float latestFilteredL = 0.0f;
static float latestRotatedRp = 0.0f;
static float latestRotatedL = 0.0f;

// Rotation parameters set by `calibrate` (or manually via `angle`). The
// rotation is applied about (rotationCenterRp, rotationCenterL); rotation is
// only applied when `rotationEnabled` is true.
static float rotationAngleRad = 0.0f;
static float rotationCenterRp = 0.0f;
static float rotationCenterL = 0.0f;
static bool rotationEnabled = false;

// Resize the active moving-average window. Clamped to [1, kMaxFilterWindow];
// window=1 disables smoothing.
void setFilterWindow(size_t window) {
  if (window < 1) window = 1;
  if (window > kMaxFilterWindow) window = kMaxFilterWindow;
  filterWindow = window;
}

size_t getFilterWindow(void) { return filterWindow; }

// Push one (rp_in, l_in) into the ring and emit the moving average of the
// most recent `filterWindow` samples. Until the ring has filled to that
// window, the average is taken over however many samples we have so we get
// a useful output immediately rather than waiting for warmup.
static void filterSample(float rp_in, float l_in, float& rp_out, float& l_out) {
  rpRing[ringHead] = rp_in;
  lRing[ringHead] = l_in;
  ringHead = (ringHead + 1) % kMaxFilterWindow;
  if (ringFilled < kMaxFilterWindow) ringFilled++;

  // Walk back from the most-recent sample, summing `n` values.
  size_t n = filterWindow < ringFilled ? filterWindow : ringFilled;
  float sumRp = 0.0f;
  float sumL = 0.0f;
  for (size_t k = 0; k < n; ++k) {
    size_t idx = (ringHead + kMaxFilterWindow - 1 - k) % kMaxFilterWindow;
    sumRp += rpRing[idx];
    sumL += lRing[idx];
  }
  rp_out = sumRp / (float)n;
  l_out = sumL / (float)n;
  latestFilteredRp = rp_out;
  latestFilteredL = l_out;
}

float getLatestFilteredRp(void) { return latestFilteredRp; }
float getLatestFilteredL(void) { return latestFilteredL; }
float getLatestRotatedRp(void) { return latestRotatedRp; }
float getLatestRotatedL(void) { return latestRotatedL; }

// Index into the rotated history by age: samples_ago=0 is the newest sample,
// 1 is the previous one, etc. Returns false if the requested age isn't yet
// available (warmup) or the request is out of range.
bool getRecentRotatedSample(size_t samples_ago, float* rp, float* l) {
  if (rp == nullptr || l == nullptr) {
    return false;
  }
  if (measurementCount == 0 || samples_ago >= measurementCount) {
    return false;
  }

  const size_t idx = measurementCount - 1 - samples_ago;
  *rp = rotatedRpArray[idx];
  *l = rotatedLArray[idx];
  return true;
}

// Standard 2D rotation by `angle_rad` about the origin. Callers that want a
// rotation about a non-origin center should translate, rotate here, then
// translate back (see appendMeasurement for the canonical pattern).
void rotateSample(float rp_in, float l_in, float angle_rad,
                  float& rp_out, float& l_out) {
  float c = cosf(angle_rad);
  float s = sinf(angle_rad);
  rp_out = rp_in * c - l_in * s;
  l_out = rp_in * s + l_in * c;
}

// Rotation knobs. Normally set by calibration.cpp (via the `calibrate` serial
// command), but also reachable directly from `angle`/`rotated` for manual
// override and from crack_detection.cpp's parabola fit (read-only via
// getRotationCenter).
void setRotationAngle(float angle_rad) { rotationAngleRad = angle_rad; }
float getRotationAngle(void) { return rotationAngleRad; }
void setRotationCenter(float rp_center, float l_center) {
  rotationCenterRp = rp_center;
  rotationCenterL = l_center;
}
void getRotationCenter(float* rp_center, float* l_center) {
  if (rp_center != nullptr) {
    *rp_center = rotationCenterRp;
  }
  if (l_center != nullptr) {
    *l_center = rotationCenterL;
  }
}
void setRotationEnabled(bool enabled) { rotationEnabled = enabled; }
bool getRotationEnabled(void) { return rotationEnabled; }

// Main entry point called every tick: smooth, rotate, store.
void appendMeasurement(float rp_ohms, float l_uH) {
  // Step 1: smooth into the rolling moving average.
  float rp_f, l_f;
  filterSample(rp_ohms, l_uH, rp_f, l_f);

  // Step 2: rotate about the calibrated center if enabled. We translate to
  // the center, rotate, then translate back so the geometric meaning is
  // "rotation about (rotationCenterRp, rotationCenterL)".
  if (rotationEnabled) {
    float rpShifted = rp_f - rotationCenterRp;
    float lShifted = l_f - rotationCenterL;
    rotateSample(rpShifted, lShifted, rotationAngleRad, latestRotatedRp, latestRotatedL);
    latestRotatedRp += rotationCenterRp;
    latestRotatedL += rotationCenterL;
  } else {
    // Pass-through so getLatestRotated* stays sensible even before calibrate.
    latestRotatedRp = rp_f;
    latestRotatedL = l_f;
  }

  // Step 3: store in the history. Until we fill it the first time, we just
  // append; after that we shift left to keep the newest `array_length`. The
  // O(N) shift is fine at our sample rate but is the main thing to revisit
  // if this ever scales up.
  if (measurementCount < array_length) {
    Rp_array[measurementCount] = rp_f;
    L_array[measurementCount] = l_f;
    rotatedRpArray[measurementCount] = latestRotatedRp;
    rotatedLArray[measurementCount] = latestRotatedL;
    measurementCount++;
    return;
  }

  for (size_t i = 1; i < array_length; ++i) {
    Rp_array[i - 1] = Rp_array[i];
    L_array[i - 1] = L_array[i];
    rotatedRpArray[i - 1] = rotatedRpArray[i];
    rotatedLArray[i - 1] = rotatedLArray[i];
  }

  Rp_array[array_length - 1] = rp_f;
  L_array[array_length - 1] = l_f;
  rotatedRpArray[array_length - 1] = latestRotatedRp;
  rotatedLArray[array_length - 1] = latestRotatedL;
}

// Mean of the most-recent `requested_samples` filtered (pre-rotation) samples.
// `requested_samples == 0` or larger than what we have means "use everything
// available." Writes the actual count used into *used_samples if provided.
// Returns false when no samples have been collected yet.
bool getRecentMeasurementMean(size_t requested_samples, float* mean_rp,
                              float* mean_l, size_t* used_samples) {
  if (measurementCount == 0 || mean_rp == nullptr || mean_l == nullptr) {
    if (used_samples != nullptr) {
      *used_samples = 0;
    }
    return false;
  }

  size_t sampleCount = requested_samples;
  if (sampleCount == 0 || sampleCount > measurementCount) {
    sampleCount = measurementCount;
  }

  size_t start = measurementCount - sampleCount;
  float sumRp = 0.0f;
  float sumL = 0.0f;
  for (size_t i = start; i < measurementCount; ++i) {
    sumRp += Rp_array[i];
    sumL += L_array[i];
  }

  *mean_rp = sumRp / (float)sampleCount;
  *mean_l = sumL / (float)sampleCount;
  if (used_samples != nullptr) {
    *used_samples = sampleCount;
  }
  return true;
}

// Estimate the substrate's dominant trend direction in the (Rp, L) plane
// using the most recent `requested_samples` filtered samples. We use the
// least-squares slope of L vs Rp (cov(Rp,L) / var(Rp)) instead of full PCA —
// they agree to first order when the cloud is elongated and var(Rp) > 0, and
// this is much cheaper. Returns NaN if no samples have arrived or the Rp
// variance is zero (degenerate cloud). The caller (calibration.cpp) negates
// the result and stores it as rotationAngle, which lays the trend along the
// new x-axis.
float calculateDominantAngleRecent(size_t requested_samples, size_t* used_samples) {
  if (measurementCount == 0) {
    if (used_samples != nullptr) {
      *used_samples = 0;
    }
    return NAN;
  }

  size_t sampleCount = requested_samples;
  if (sampleCount == 0 || sampleCount > measurementCount) {
    sampleCount = measurementCount;
  }

  if (used_samples != nullptr) {
    *used_samples = sampleCount;
  }

  size_t start = measurementCount - sampleCount;

  // First pass: means.
  float meanRp = 0.0f;
  float meanL = 0.0f;

  for (size_t i = start; i < measurementCount; i++) {
    meanRp += Rp_array[i];
    meanL += L_array[i];
  }

  meanRp /= (float)sampleCount;
  meanL /= (float)sampleCount;

  // Second pass: var(Rp) and cov(Rp, L) around the means.
  float covRpL = 0.0f;
  float varRp = 0.0f;

  for (size_t i = start; i < measurementCount; i++) {
    float dx = Rp_array[i] - meanRp;
    float dy = L_array[i] - meanL;

    varRp += dx * dx;
    covRpL += dx * dy;
  }

  if (varRp <= 0.0f) {
    return NAN;
  }

  // slope = cov(Rp,L) / var(Rp); atan of that is the trend's angle off the
  // Rp axis. Returned in radians.
  float slope = covRpL / varRp;
  float theta = atanf(slope);

  return theta;
}
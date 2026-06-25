// Line-oriented serial CLI for runtime configuration.
//
// Architecture:
//   - serialCommandsInit() seeds the live config + state and pushes the
//     starting sensor parameters to the chip via configureSensor().
//   - serialCommandsPoll() is called every loop() tick and assembles
//     characters into lines; on '\n', the line is dispatched to processCommand.
//   - processCommand() is a flat strcmp() dispatch. Each command block is
//     small enough that a giant switch isn't worth the indirection.
//
// Conventions (kept in sync with printStatus() and printHelp()):
//   - "set" forms write a new value, then echo the post-clamp value.
//   - "show" forms (no args) just echo the current value.
//   - On/off toggles return "OK <name> on|off"; numeric set+show return "<name>=<val>".
//   - Every register-affecting setter calls configureSensor() so the chip
//     never drifts out of sync with gConfig/gState. This is the convention
//     CLAUDE.md refers to.

#include "serial_commands.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "LED.h"
#include "calibration.h"
#include "cmd_reply.h"
#include "crack_detection.h"
#include "measurement_arrays.h"
#include "memory.h"

namespace {

// Live config + state. printStatus() reads these; the per-command set blocks
// mutate them; main.cpp reads via serialCommandsGetConfig/State each tick.
static serial_command_config_t gConfig;
static serial_command_state_t gState;
static bool gInitialized = false;

static constexpr size_t kCommandBufferLen = 96;
static constexpr long kMaxCalibrationSamples = 50000;
static constexpr long kMaxReadingDelayMs = 60000;
// Commands accept µH / pF for convenience; sensor config is stored in SI
// (Henries / Farads). These factors are the conversion in both directions.
static constexpr float kMicroToBase = 1.0e-6f;
static constexpr float kPicoToBase = 1.0e-12f;
// Line assembly buffer for serialCommandsPoll().
static char gCommandBuffer[kCommandBufferLen];
static size_t gCommandLength = 0;

// Vibrate (calibration liftoff dither) parameters + a one-shot request flag the
// main loop consumes via serialCommandsTakeVibrateRequest(). Defaults are the
// starting point for tuning; change them live with `vibrate <strength> <freq>
// <duration_ms>`, or edit these to change the boot defaults.
static constexpr int16_t kDefaultVibrateStrength = 180;     // PWM per tread (0..255)
static constexpr float kDefaultVibrateFreqHz = 20.0f;       // twist reversals/sec
static constexpr uint32_t kDefaultVibrateDurationMs = 1500;
static int16_t gVibrateStrength = kDefaultVibrateStrength;
static float gVibrateFreqHz = kDefaultVibrateFreqHz;
static uint32_t gVibrateDurationMs = kDefaultVibrateDurationMs;
static bool gVibratePending = false;

const char* modeToString(ldc1101_mode_t mode) {
  return mode == LDC1101_MODE_LHR ? "lhr" : "lrp";
}

const char* speedToString(ldc_speed_mode_t speed) {
  switch (speed) {
    case LDC_SPEED_ACCURACY_MAX: return "accuracy";
    case LDC_SPEED_BALANCED_1: return "balanced1";
    case LDC_SPEED_BALANCED_2: return "balanced2";
    case LDC_SPEED_FAST:
    default:
      return "fast";
  }
}

// Re-push the current sensor parameters + mode/speed to the LDC1101.
// Called from serialCommandsInit() and from any command block that changes a
// register-affecting field. Cheap (just a few SPI writes) so we don't bother
// diffing.
void configureSensor() {
  ldc1101_configure(gConfig.sensor_l_h, gConfig.sensor_c_f, gConfig.sensor_q,
      gState.mode, gState.speed_mode,
      gConfig.switch_enable, gConfig.switch_gpio);
}

// Help and status mirror each other: same fields, same order, same names.
// Each `<name>` command shown here corresponds 1:1 to a `<name>=...` token
// printed by `status`.
void printHelp() {
  Reply.println("commands:");
  Reply.println("  help                       // show this list");
  Reply.println("  status                     // print all current values");

  // Sensor parameters.
  Reply.println("  l_h [uH]                   // sensor inductance");
  Reply.println("  c_f [pF]                   // sensor capacitance");
  Reply.println("  q [ratio]                  // sensor quality factor");

  // Reading mode.
  Reply.println("  mode lrp|lhr               // measurement mode");
  Reply.println("  speed accuracy|balanced1|balanced2|fast");
  Reply.println("  stream on|off              // streaming output on/off");
  Reply.println("  delay <ms>                 // sample period");

  // Signal processing / calibration.
  Reply.println("  smoothing <n>              // moving-average window");
  Reply.println("  rotated on|off             // apply calibrated rotation");
  Reply.println("  angle [radians]            // get/set rotation angle");
  Reply.println("  calibrate [samples]        // run PCA calibration on recent samples");
  Reply.println("  baseline [samples]         // re-anchor rotation center to recent flat signal");
  Reply.println("  vibrate [str freq_hz ms]   // pulse treads opposite to dither liftoff");

  // Crack detection tuning.
  Reply.println("  crack_window [n]           // parabola fit window samples");
  Reply.println("  crack_threshold [val]      // min parabola peak above baseline");
  Reply.println("  crack_r2 [0..1]            // min parabola R^2 fit");
  Reply.println("  crack_deviation [rad]      // max angular tilt of (t,Rp,L) curve off the t-L plane, 0..pi/2");
  Reply.println("  crack_scale [scale]        // peak->length scale factor");
  Reply.println("  crack_output on|off        // emit per-crack >mag >crack_x >crack_size >width");
  Reply.println("  crack_debug on|off         // print crack detector debug state");

  // Persistent per-material profiles (saved to NVS flash).
  Reply.println("  save <material>            // store all settings under a name");
  Reply.println("  retrieve <material>        // load settings saved under a name");
  Reply.println("  materials                  // list saved materials + their rotation angle");
  Reply.println("  get_material               // name the saved material whose angle is closest to the current calibration");
  Reply.println("  forget <material>          // delete a saved material");
}

void printStatus() {
  // Sensor parameters. Print in the same units the commands accept.
  Reply.print("l_h=");
  Reply.print(gConfig.sensor_l_h / kMicroToBase, 6);
  Reply.print(" c_f=");
  Reply.print(gConfig.sensor_c_f / kPicoToBase, 6);
  Reply.print(" q=");
  Reply.println(gConfig.sensor_q, 6);

  // Reading mode.
  Reply.print("mode=");
  Reply.print(modeToString(gState.mode));
  Reply.print(" speed=");
  Reply.print(speedToString(gState.speed_mode));
  Reply.print(" stream=");
  Reply.print(gState.streaming_enabled ? "on" : "off");
  Reply.print(" delay=");
  Reply.println((unsigned long)gState.reading_delay_ms);

  // Signal processing.
  Reply.print("smoothing=");
  Reply.print((unsigned int)getFilterWindow());
  Reply.print(" rotated=");
  Reply.print(gState.rotated ? "on" : "off");
  Reply.print(" angle=");
  Reply.println(getRotationAngle(), 6);

  // Crack detection.
  Reply.print("crack_window=");
  Reply.print((unsigned int)crackDetectionGetWindowSamples());
  Reply.print(" crack_threshold=");
  Reply.print(crackDetectionGetThreshold(), 6);
  Reply.print(" crack_r2=");
  Reply.println(crackDetectionGetMinParabolaR2(), 6);

  Reply.print("crack_deviation=");
  Reply.print(crackDetectionGetMaxDeviationRad(), 6);
  Reply.print(" crack_scale=");
  Reply.println(crackDetectionGetLengthEstimateScale(), 6);

  Reply.print("crack_output=");
  Reply.print(gState.crack_output ? "on" : "off");
  Reply.print(" crack_debug=");
  Reply.println(gState.crack_debug_output ? "on" : "off");
}

// Dispatch one user-typed line. Format: <command> [arg ...]. Whitespace-only
// lines are ignored. Each command block uses the strtok cursor left by the
// initial split. Each block validates its own arguments and prints either an
// "ERR ..." line on bad input or an echo of the post-clamp value on success.
//
// Numeric commands generally accept "no args = show current; one arg = set
// then show." Boolean commands always require "on|off".
void processCommand(char* line) {
  // Strip leading whitespace; treat empty lines as no-ops.
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  if (*line == '\0') {return;}

  // First token is the command name; rest are consumed inside the matching block.
  char* token = strtok(line, " \t");
  if (token == nullptr) {return;}

  if (strcmp(token, "help") == 0) {printHelp(); return;}

  if (strcmp(token, "status") == 0) {printStatus(); return;}

  // --- Sensor parameters (l_h, c_f, q) -------------------------------------
  // Stored in SI internally; the CLI takes / shows µH and pF respectively. Any
  // change is pushed to the chip immediately via configureSensor() because
  // these values are inputs to the RP_SET / TC1 / TC2 / DIG_CONF formulas.

  if (strcmp(token, "l_h") == 0 || strcmp(token, "lh") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed <= 0.0f) {
        Reply.println("ERR l_h must be > 0 (uH)");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Reply.println("ERR usage: l_h [uH]");
        return;
      }
      gConfig.sensor_l_h = parsed * kMicroToBase;
      configureSensor();
    }

    Reply.print("l_h=");
    Reply.println(gConfig.sensor_l_h / kMicroToBase, 6);
    return;
  }

  if (strcmp(token, "c_f") == 0 || strcmp(token, "cf") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed <= 0.0f) {
        Reply.println("ERR c_f must be > 0 (pF)");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Reply.println("ERR usage: c_f [pF]");
        return;
      }
      gConfig.sensor_c_f = parsed * kPicoToBase;
      configureSensor();
    }

    Reply.print("c_f=");
    Reply.println(gConfig.sensor_c_f / kPicoToBase, 6);
    return;
  }

  if (strcmp(token, "q") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed <= 0.0f) {
        Reply.println("ERR q must be > 0");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Reply.println("ERR usage: q [value]");
        return;
      }
      gConfig.sensor_q = parsed;
      configureSensor();
    }

    Reply.print("q=");
    Reply.println(gConfig.sensor_q, 6);
    return;
  }

  // --- Signal processing ---------------------------------------------------
  // angle/rotated/calibrate share the rotation state owned by measurement_arrays.
  // `angle` sets just the rotation angle (leaving center alone). Use `calibrate`
  // to recompute both from current data when you change substrate.

  if (strcmp(token, "angle") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0') {
        Reply.println("ERR usage: angle [radians]");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Reply.println("ERR usage: angle [radians]");
        return;
      }
      setRotationAngle(parsed);
    }

    Reply.print("angle=");
    Reply.println(getRotationAngle(), 6);
    return;
  }

  // --- Reading mode (stream/delay) ----------------------------------------
  // Don't push to the chip — they only affect the main loop's emit timing.

  if (strcmp(token, "stream") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Reply.println("ERR usage: stream on|off");
      return;
    }
    if (strcmp(value, "on") == 0) {
      gState.streaming_enabled = true;
      Reply.println("OK stream on");
      return;
    }
    if (strcmp(value, "off") == 0) {
      gState.streaming_enabled = false;
      Reply.println("OK stream off");
      return;
    }
    Reply.println("ERR usage: stream on|off");
    return;
  }

  if (strcmp(token, "delay") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Reply.println("ERR usage: delay <ms>");
      return;
    }

    long parsed = strtol(value, nullptr, 10);
    if (parsed <= 0 || parsed > kMaxReadingDelayMs) {
      Reply.print("ERR delay must be 1..");
      Reply.println(kMaxReadingDelayMs);
      return;
    }

    gState.reading_delay_ms = (uint32_t)parsed;
    Reply.print("OK delay ");
    Reply.println((unsigned long)gState.reading_delay_ms);
    return;
  }

  if (strcmp(token, "smoothing") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Reply.println("ERR usage: smoothing <n>");
      return;
    }

    long parsed = strtol(value, nullptr, 10);
    if (parsed <= 0) {
      Reply.println("ERR smoothing must be >= 1");
      return;
    }

    setFilterWindow((size_t)parsed);
    Reply.print("OK smoothing ");
    Reply.println((unsigned int)getFilterWindow());
    return;
  }

  // --- Crack detection tuning ---------------------------------------------
  // All of these write through to crack_detection; the value is re-clamped on
  // the way in, and the echo prints the post-clamp value.

  if (strcmp(token, "crack_window") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      long parsed = strtol(value, nullptr, 10);
      if (parsed <= 0) {
        Reply.println("ERR crack_window must be >= 1");
        return;
      }
      crackDetectionSetWindowSamples((size_t)parsed);
    }

    Reply.print("crack_window=");
    Reply.println((unsigned int)crackDetectionGetWindowSamples());
    return;
  }

  if (strcmp(token, "crack_threshold") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed < 0.0f) {
        Reply.println("ERR crack_threshold must be >= 0");
        return;
      }
      crackDetectionSetThreshold(parsed);
    }

    Reply.print("crack_threshold=");
    Reply.println(crackDetectionGetThreshold(), 6);
    return;
  }

  if (strcmp(token, "crack_r2") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed < 0.0f || parsed > 1.0f) {
        Reply.println("ERR crack_r2 must be 0..1");
        return;
      }
      crackDetectionSetMinParabolaR2(parsed);
    }

    Reply.print("crack_r2=");
    Reply.println(crackDetectionGetMinParabolaR2(), 6);
    return;
  }

  if (strcmp(token, "crack_deviation") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed < 0.0f) {
        Reply.println("ERR crack_deviation must be 0..pi/2 (rad)");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Reply.println("ERR usage: crack_deviation [angle_rad]");
        return;
      }
      crackDetectionSetMaxDeviationRad(parsed);
    }

    Reply.print("crack_deviation=");
    Reply.println(crackDetectionGetMaxDeviationRad(), 6);
    return;
  }

  if (strcmp(token, "crack_scale") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed < 0.0f) {
        Reply.println("ERR crack_scale must be >= 0");
        return;
      }
      crackDetectionSetLengthEstimateScale(parsed);
    }

    Reply.print("crack_scale=");
    Reply.println(crackDetectionGetLengthEstimateScale(), 6);
    return;
  }

  if (strcmp(token, "crack_debug") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Reply.println("ERR usage: crack_debug on|off");
      return;
    }
    if (strcmp(value, "on") == 0) {
      gState.crack_debug_output = true;
      Reply.println("OK crack_debug on");
      return;
    }
    if (strcmp(value, "off") == 0) {
      gState.crack_debug_output = false;
      Reply.println("OK crack_debug off");
      return;
    }
    Reply.println("ERR usage: crack_debug on|off");
    return;
  }

  if (strcmp(token, "crack_output") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Reply.println("ERR usage: crack_output on|off");
      return;
    }
    if (strcmp(value, "on") == 0) {
      gState.crack_output = true;
      Reply.println("OK crack_output on");
      return;
    }
    if (strcmp(value, "off") == 0) {
      gState.crack_output = false;
      Reply.println("OK crack_output off");
      return;
    }
    Reply.println("ERR usage: crack_output on|off");
    return;
  }

  // --- Mode/speed selection (register-affecting) --------------------------
  // Both call configureSensor() because they change DIG_CONF / TC* register
  // values that the driver recomputes from the current config.

  if (strcmp(token, "mode") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Reply.println("ERR usage: mode lrp|lhr");
      return;
    }

    ldc1101_mode_t newMode;
    if (strcmp(value, "lrp") == 0) {
      newMode = LDC1101_MODE_RP_L;
    } else if (strcmp(value, "lhr") == 0) {
      newMode = LDC1101_MODE_LHR;
    } else {
      Reply.println("ERR usage: mode lrp|lhr");
      return;
    }

    gState.mode = newMode;
    configureSensor();
    Reply.print("OK mode ");
    Reply.println(modeToString(gState.mode));
    return;
  }

  if (strcmp(token, "speed") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Reply.println("ERR usage: speed accuracy|balanced1|balanced2|fast");
      return;
    }

    ldc_speed_mode_t newSpeed;
    if (strcmp(value, "accuracy") == 0) {
      newSpeed = LDC_SPEED_ACCURACY_MAX;
    } else if (strcmp(value, "balanced1") == 0 || strcmp(value, "bal1") == 0) {
      newSpeed = LDC_SPEED_BALANCED_1;
    } else if (strcmp(value, "balanced2") == 0 || strcmp(value, "bal2") == 0) {
      newSpeed = LDC_SPEED_BALANCED_2;
    } else if (strcmp(value, "fast") == 0) {
      newSpeed = LDC_SPEED_FAST;
    } else {
      Reply.println("ERR usage: speed accuracy|balanced1|balanced2|fast");
      return;
    }

    gState.speed_mode = newSpeed;
    configureSensor();
    Reply.print("OK speed ");
    Reply.println(speedToString(gState.speed_mode));
    return;
  }

  // --- Per-substrate calibration ------------------------------------------
  // Runs PCA on the most recent N filtered samples and installs the resulting
  // rotation. LED flashes during the run so the operator can confirm timing.

  if (strcmp(token, "calibrate") == 0) {
    char* value = strtok(nullptr, " \t");
    size_t sampleCount = 40;
    if (value != nullptr) {
      long parsed = strtol(value, nullptr, 10);
      if (parsed <= 0 || parsed > kMaxCalibrationSamples) {
        Reply.print("ERR calibrate samples must be 1..");
        Reply.println(kMaxCalibrationSamples);
        return;
      }
      sampleCount = (size_t)parsed;
    }
    ledFlash(10, 30);
    Reply.print("calibrate start samples=");
    Reply.println((unsigned int)sampleCount);
    calibration_result_t result = calibrationRun(gConfig.sensor_c_f, sampleCount);
    calibrationPrintResult(&result);
    return;
  }

  // --- Baseline re-anchor -------------------------------------------------
  // Like `calibrate`, but only re-anchors the rotation center to the current
  // mean (Rp, L) — leaves the rotation angle alone. Use this to re-zero the
  // crack_threshold reference after thermal/baseline drift without redoing
  // the substrate-trend PCA.

  if (strcmp(token, "baseline") == 0) {
    char* value = strtok(nullptr, " \t");
    size_t sampleCount = 40;
    if (value != nullptr) {
      long parsed = strtol(value, nullptr, 10);
      if (parsed <= 0 || parsed > kMaxCalibrationSamples) {
        Reply.print("ERR baseline samples must be 1..");
        Reply.println(kMaxCalibrationSamples);
        return;
      }
      sampleCount = (size_t)parsed;
    }
    ledFlash(10, 30);
    Reply.print("baseline start samples=");
    Reply.println((unsigned int)sampleCount);
    baseline_result_t result = baselineRun(sampleCount);
    baselinePrintResult(&result);
    return;
  }

  // --- Vibrate (calibration liftoff dither) -------------------------------
  // `vibrate` triggers a burst with the current params; `vibrate <strength>
  // <freq_hz> <duration_ms>` updates all three then triggers. main.cpp does the
  // actual motor pulsing (it owns the motors); we just stage params + a flag.
  if (strcmp(token, "vibrate") == 0) {
    char* a = strtok(nullptr, " \t");
    if (a != nullptr) {
      char* b = strtok(nullptr, " \t");
      char* c = strtok(nullptr, " \t");
      if (b == nullptr || c == nullptr || strtok(nullptr, " \t") != nullptr) {
        Reply.println("ERR usage: vibrate [strength freq_hz duration_ms]");
        return;
      }
      long strength = strtol(a, nullptr, 10);
      float freq = strtof(b, nullptr);
      long duration = strtol(c, nullptr, 10);
      if (strength < 0 || strength > 255 || freq <= 0.0f || duration <= 0) {
        Reply.println("ERR vibrate: strength 0..255, freq_hz > 0, duration_ms > 0");
        return;
      }
      gVibrateStrength = (int16_t)strength;
      gVibrateFreqHz = freq;
      gVibrateDurationMs = (uint32_t)duration;
    }
    gVibratePending = true;
    Reply.print("OK vibrate strength=");
    Reply.print(gVibrateStrength);
    Reply.print(" freq_hz=");
    Reply.print(gVibrateFreqHz, 2);
    Reply.print(" duration_ms=");
    Reply.println((unsigned long)gVibrateDurationMs);
    return;
  }

  // `rotate` and `rotation` are accepted as aliases for `rotated` so the
  // command is forgiving of how the user remembers it.
  if (strcmp(token, "rotate") == 0 ||
      strcmp(token, "rotated") == 0 ||
      strcmp(token, "rotation") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Reply.println("ERR usage: rotated on|off");
      return;
    }
    if (strcmp(value, "on") == 0) {
      gState.rotated = true;
      setRotationEnabled(true);
      Reply.println("OK rotated on");
      return;
    }
    if (strcmp(value, "off") == 0) {
      gState.rotated = false;
      setRotationEnabled(false);
      Reply.println("OK rotated off");
      return;
    }
    Reply.println("ERR usage: rotated on|off");
    return;
  }

  // --- Persistent material profiles ---------------------------------------
  // `save` snapshots the full live config/state (plus the rotation and
  // crack-detector settings memory.cpp reads from their modules) into an NVS
  // namespace named after the material. `retrieve` loads it back and re-pushes
  // the sensor parameters to the chip via configureSensor(). `materials` lists
  // the saved names; `forget` deletes one.

  if (strcmp(token, "save") == 0) {
    char* name = strtok(nullptr, " \t");
    if (name == nullptr || strtok(nullptr, " \t") != nullptr) {
      Reply.println("ERR usage: save <material>");
      return;
    }
    if (memorySaveMaterial(name, &gConfig, &gState)) {
      Reply.print("OK saved ");
      Reply.println(name);
    } else {
      Reply.println("ERR save failed (bad name or storage error)");
    }
    return;
  }

  if (strcmp(token, "retrieve") == 0 || strcmp(token, "load") == 0) {
    char* name = strtok(nullptr, " \t");
    if (name == nullptr || strtok(nullptr, " \t") != nullptr) {
      Reply.println("ERR usage: retrieve <material>");
      return;
    }
    if (memoryRetrieveMaterial(name, &gConfig, &gState)) {
      // Restored sensor/mode/speed need to reach the chip; rotation and crack
      // settings were already applied inside memoryRetrieveMaterial().
      configureSensor();
      Reply.print("OK retrieved ");
      Reply.println(name);
      printStatus();
    } else {
      Reply.print("ERR no saved material: ");
      Reply.println(name);
    }
    return;
  }

  if (strcmp(token, "materials") == 0) {
    if (strtok(nullptr, " \t") != nullptr) {
      Reply.println("ERR usage: materials");
      return;
    }
    memoryListMaterials();
    return;
  }

  // `get_material` identifies the closest saved profile to the *current*
  // rotation angle — run it right after `calibrate` on an unknown substrate to
  // see which stored material it most resembles. Named distinctly from
  // `materials` (the list command). Report-only; doesn't load.
  if (strcmp(token, "get_material") == 0) {
    if (strtok(nullptr, " \t") != nullptr) {
      Reply.println("ERR usage: get_material");
      return;
    }
    memoryMatchByAngle(getRotationAngle());
    return;
  }

  if (strcmp(token, "forget") == 0) {
    char* name = strtok(nullptr, " \t");
    if (name == nullptr || strtok(nullptr, " \t") != nullptr) {
      Reply.println("ERR usage: forget <material>");
      return;
    }
    if (memoryForgetMaterial(name)) {
      Reply.print("OK forgot ");
      Reply.println(name);
    } else {
      Reply.print("ERR no saved material: ");
      Reply.println(name);
    }
    return;
  }

  Reply.print("ERR unknown command: ");
  Reply.println(token);
}

}  // namespace

// Seed the live config + state, push them to the chip, and print the help
// banner. main.cpp::setup() calls this once at boot with its `kSensor*`
// defaults; subsequent CLI commands mutate gConfig/gState in place.
void serialCommandsInit(const serial_command_config_t* config,
                        const serial_command_state_t* initial_state) {
  if (config == nullptr || initial_state == nullptr) {
    return;
  }

  gConfig = *config;
  gState = *initial_state;
  setRotationEnabled(gState.rotated);
  gCommandLength = 0;
  gInitialized = true;

  // Push the seeded sensor parameters to the LDC1101 here so main.cpp doesn't
  // have to call ldc1101_configure() separately.
  configureSensor();

  printHelp();
}

// Non-blocking line assembler — call once per loop() tick. Drains whatever
// the USB-CDC has buffered, ignores '\r', dispatches on '\n', and silently
// drops any bytes past the buffer length (typing longer than 95 chars is on
// the user). Bytes after a buffer overflow keep the partial line until '\n',
// which is then dispatched as the truncated line.
void serialCommandsPoll(void) {
  if (!gInitialized) {
    return;
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      gCommandBuffer[gCommandLength] = '\0';
      processCommand(gCommandBuffer);
      gCommandLength = 0;
      continue;
    }

    if (gCommandLength < (kCommandBufferLen - 1)) {
      gCommandBuffer[gCommandLength++] = c;
    }
  }
}

// Dispatch a command line from a non-USB source (the Serial1 link). processCommand
// tokenizes in place with strtok, so copy into a local buffer first.
void serialCommandsProcessLine(const char* line) {
  if (!gInitialized || line == nullptr) {
    return;
  }
  char buf[kCommandBufferLen];
  strncpy(buf, line, kCommandBufferLen - 1);
  buf[kCommandBufferLen - 1] = '\0';
  // Frame this command's output back over Serial1 so the PC app can show it.
  Reply.beginBridgeCapture();
  processCommand(buf);
  Reply.endBridgeCapture();
}

// One-shot pull of a pending `vibrate` request. Returns true exactly once per
// `vibrate` command, writing the staged params; main.cpp starts the burst.
bool serialCommandsTakeVibrateRequest(int16_t* strength, float* freq_hz,
                                      uint32_t* duration_ms) {
  if (!gVibratePending) {
    return false;
  }
  gVibratePending = false;
  if (strength != nullptr) *strength = gVibrateStrength;
  if (freq_hz != nullptr) *freq_hz = gVibrateFreqHz;
  if (duration_ms != nullptr) *duration_ms = gVibrateDurationMs;
  return true;
}

// Snapshots of the live state for main.cpp. Returned by value (small structs)
// so callers can't accidentally mutate our globals through the result.
serial_command_state_t serialCommandsGetState(void) {
  return gState;
}

serial_command_config_t serialCommandsGetConfig(void) {
  return gConfig;
}

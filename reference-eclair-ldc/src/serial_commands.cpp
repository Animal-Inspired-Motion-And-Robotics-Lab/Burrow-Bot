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
  Serial.println("commands:");
  Serial.println("  help                       // show this list");
  Serial.println("  status                     // print all current values");

  // Sensor parameters.
  Serial.println("  l_h [uH]                   // sensor inductance");
  Serial.println("  c_f [pF]                   // sensor capacitance");
  Serial.println("  q [ratio]                  // sensor quality factor");

  // Reading mode.
  Serial.println("  mode lrp|lhr               // measurement mode");
  Serial.println("  speed accuracy|balanced1|balanced2|fast");
  Serial.println("  stream on|off              // streaming output on/off");
  Serial.println("  delay <ms>                 // sample period");

  // Signal processing / calibration.
  Serial.println("  smoothing <n>              // moving-average window");
  Serial.println("  rotated on|off             // apply calibrated rotation");
  Serial.println("  angle [radians]            // get/set rotation angle");
  Serial.println("  calibrate [samples]        // run PCA calibration on recent samples");
  Serial.println("  baseline [samples]         // re-anchor rotation center to recent flat signal");

  // Crack detection tuning.
  Serial.println("  crack_window [n]           // parabola fit window samples");
  Serial.println("  crack_threshold [val]      // min parabola peak above baseline");
  Serial.println("  crack_r2 [0..1]            // min parabola R^2 fit");
  Serial.println("  crack_deviation [rad]      // max angular tilt of (t,Rp,L) curve off the t-L plane, 0..pi/2");
  Serial.println("  crack_scale [scale]        // peak->length scale factor");
  Serial.println("  crack_output on|off        // emit per-crack >mag >crack_x >crack_size >width");
  Serial.println("  crack_debug on|off         // print crack detector debug state");

  // Persistent per-material profiles (saved to NVS flash).
  Serial.println("  save <material>            // store all settings under a name");
  Serial.println("  retrieve <material>        // load settings saved under a name");
  Serial.println("  materials                  // list saved materials + their rotation angle");
  Serial.println("  get_material               // name the saved material whose angle is closest to the current calibration");
  Serial.println("  forget <material>          // delete a saved material");
}

void printStatus() {
  // Sensor parameters. Print in the same units the commands accept.
  Serial.print("l_h=");
  Serial.print(gConfig.sensor_l_h / kMicroToBase, 6);
  Serial.print(" c_f=");
  Serial.print(gConfig.sensor_c_f / kPicoToBase, 6);
  Serial.print(" q=");
  Serial.println(gConfig.sensor_q, 6);

  // Reading mode.
  Serial.print("mode=");
  Serial.print(modeToString(gState.mode));
  Serial.print(" speed=");
  Serial.print(speedToString(gState.speed_mode));
  Serial.print(" stream=");
  Serial.print(gState.streaming_enabled ? "on" : "off");
  Serial.print(" delay=");
  Serial.println((unsigned long)gState.reading_delay_ms);

  // Signal processing.
  Serial.print("smoothing=");
  Serial.print((unsigned int)getFilterWindow());
  Serial.print(" rotated=");
  Serial.print(gState.rotated ? "on" : "off");
  Serial.print(" angle=");
  Serial.println(getRotationAngle(), 6);

  // Crack detection.
  Serial.print("crack_window=");
  Serial.print((unsigned int)crackDetectionGetWindowSamples());
  Serial.print(" crack_threshold=");
  Serial.print(crackDetectionGetThreshold(), 6);
  Serial.print(" crack_r2=");
  Serial.println(crackDetectionGetMinParabolaR2(), 6);

  Serial.print("crack_deviation=");
  Serial.print(crackDetectionGetMaxDeviationRad(), 6);
  Serial.print(" crack_scale=");
  Serial.println(crackDetectionGetLengthEstimateScale(), 6);

  Serial.print("crack_output=");
  Serial.print(gState.crack_output ? "on" : "off");
  Serial.print(" crack_debug=");
  Serial.println(gState.crack_debug_output ? "on" : "off");
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
        Serial.println("ERR l_h must be > 0 (uH)");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Serial.println("ERR usage: l_h [uH]");
        return;
      }
      gConfig.sensor_l_h = parsed * kMicroToBase;
      configureSensor();
    }

    Serial.print("l_h=");
    Serial.println(gConfig.sensor_l_h / kMicroToBase, 6);
    return;
  }

  if (strcmp(token, "c_f") == 0 || strcmp(token, "cf") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed <= 0.0f) {
        Serial.println("ERR c_f must be > 0 (pF)");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Serial.println("ERR usage: c_f [pF]");
        return;
      }
      gConfig.sensor_c_f = parsed * kPicoToBase;
      configureSensor();
    }

    Serial.print("c_f=");
    Serial.println(gConfig.sensor_c_f / kPicoToBase, 6);
    return;
  }

  if (strcmp(token, "q") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed <= 0.0f) {
        Serial.println("ERR q must be > 0");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Serial.println("ERR usage: q [value]");
        return;
      }
      gConfig.sensor_q = parsed;
      configureSensor();
    }

    Serial.print("q=");
    Serial.println(gConfig.sensor_q, 6);
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
        Serial.println("ERR usage: angle [radians]");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Serial.println("ERR usage: angle [radians]");
        return;
      }
      setRotationAngle(parsed);
    }

    Serial.print("angle=");
    Serial.println(getRotationAngle(), 6);
    return;
  }

  // --- Reading mode (stream/delay) ----------------------------------------
  // Don't push to the chip — they only affect the main loop's emit timing.

  if (strcmp(token, "stream") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: stream on|off");
      return;
    }
    if (strcmp(value, "on") == 0) {
      gState.streaming_enabled = true;
      Serial.println("OK stream on");
      return;
    }
    if (strcmp(value, "off") == 0) {
      gState.streaming_enabled = false;
      Serial.println("OK stream off");
      return;
    }
    Serial.println("ERR usage: stream on|off");
    return;
  }

  if (strcmp(token, "delay") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: delay <ms>");
      return;
    }

    long parsed = strtol(value, nullptr, 10);
    if (parsed <= 0 || parsed > kMaxReadingDelayMs) {
      Serial.print("ERR delay must be 1..");
      Serial.println(kMaxReadingDelayMs);
      return;
    }

    gState.reading_delay_ms = (uint32_t)parsed;
    Serial.print("OK delay ");
    Serial.println((unsigned long)gState.reading_delay_ms);
    return;
  }

  if (strcmp(token, "smoothing") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: smoothing <n>");
      return;
    }

    long parsed = strtol(value, nullptr, 10);
    if (parsed <= 0) {
      Serial.println("ERR smoothing must be >= 1");
      return;
    }

    setFilterWindow((size_t)parsed);
    Serial.print("OK smoothing ");
    Serial.println((unsigned int)getFilterWindow());
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
        Serial.println("ERR crack_window must be >= 1");
        return;
      }
      crackDetectionSetWindowSamples((size_t)parsed);
    }

    Serial.print("crack_window=");
    Serial.println((unsigned int)crackDetectionGetWindowSamples());
    return;
  }

  if (strcmp(token, "crack_threshold") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed < 0.0f) {
        Serial.println("ERR crack_threshold must be >= 0");
        return;
      }
      crackDetectionSetThreshold(parsed);
    }

    Serial.print("crack_threshold=");
    Serial.println(crackDetectionGetThreshold(), 6);
    return;
  }

  if (strcmp(token, "crack_r2") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed < 0.0f || parsed > 1.0f) {
        Serial.println("ERR crack_r2 must be 0..1");
        return;
      }
      crackDetectionSetMinParabolaR2(parsed);
    }

    Serial.print("crack_r2=");
    Serial.println(crackDetectionGetMinParabolaR2(), 6);
    return;
  }

  if (strcmp(token, "crack_deviation") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed < 0.0f) {
        Serial.println("ERR crack_deviation must be 0..pi/2 (rad)");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Serial.println("ERR usage: crack_deviation [angle_rad]");
        return;
      }
      crackDetectionSetMaxDeviationRad(parsed);
    }

    Serial.print("crack_deviation=");
    Serial.println(crackDetectionGetMaxDeviationRad(), 6);
    return;
  }

  if (strcmp(token, "crack_scale") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed < 0.0f) {
        Serial.println("ERR crack_scale must be >= 0");
        return;
      }
      crackDetectionSetLengthEstimateScale(parsed);
    }

    Serial.print("crack_scale=");
    Serial.println(crackDetectionGetLengthEstimateScale(), 6);
    return;
  }

  if (strcmp(token, "crack_debug") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: crack_debug on|off");
      return;
    }
    if (strcmp(value, "on") == 0) {
      gState.crack_debug_output = true;
      Serial.println("OK crack_debug on");
      return;
    }
    if (strcmp(value, "off") == 0) {
      gState.crack_debug_output = false;
      Serial.println("OK crack_debug off");
      return;
    }
    Serial.println("ERR usage: crack_debug on|off");
    return;
  }

  if (strcmp(token, "crack_output") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: crack_output on|off");
      return;
    }
    if (strcmp(value, "on") == 0) {
      gState.crack_output = true;
      Serial.println("OK crack_output on");
      return;
    }
    if (strcmp(value, "off") == 0) {
      gState.crack_output = false;
      Serial.println("OK crack_output off");
      return;
    }
    Serial.println("ERR usage: crack_output on|off");
    return;
  }

  // --- Mode/speed selection (register-affecting) --------------------------
  // Both call configureSensor() because they change DIG_CONF / TC* register
  // values that the driver recomputes from the current config.

  if (strcmp(token, "mode") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: mode lrp|lhr");
      return;
    }

    ldc1101_mode_t newMode;
    if (strcmp(value, "lrp") == 0) {
      newMode = LDC1101_MODE_RP_L;
    } else if (strcmp(value, "lhr") == 0) {
      newMode = LDC1101_MODE_LHR;
    } else {
      Serial.println("ERR usage: mode lrp|lhr");
      return;
    }

    gState.mode = newMode;
    configureSensor();
    Serial.print("OK mode ");
    Serial.println(modeToString(gState.mode));
    return;
  }

  if (strcmp(token, "speed") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: speed accuracy|balanced1|balanced2|fast");
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
      Serial.println("ERR usage: speed accuracy|balanced1|balanced2|fast");
      return;
    }

    gState.speed_mode = newSpeed;
    configureSensor();
    Serial.print("OK speed ");
    Serial.println(speedToString(gState.speed_mode));
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
        Serial.print("ERR calibrate samples must be 1..");
        Serial.println(kMaxCalibrationSamples);
        return;
      }
      sampleCount = (size_t)parsed;
    }
    ledFlash(10, 30);
    Serial.print("calibrate start samples=");
    Serial.println((unsigned int)sampleCount);
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
        Serial.print("ERR baseline samples must be 1..");
        Serial.println(kMaxCalibrationSamples);
        return;
      }
      sampleCount = (size_t)parsed;
    }
    ledFlash(10, 30);
    Serial.print("baseline start samples=");
    Serial.println((unsigned int)sampleCount);
    baseline_result_t result = baselineRun(sampleCount);
    baselinePrintResult(&result);
    return;
  }

  // `rotate` and `rotation` are accepted as aliases for `rotated` so the
  // command is forgiving of how the user remembers it.
  if (strcmp(token, "rotate") == 0 ||
      strcmp(token, "rotated") == 0 ||
      strcmp(token, "rotation") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: rotated on|off");
      return;
    }
    if (strcmp(value, "on") == 0) {
      gState.rotated = true;
      setRotationEnabled(true);
      Serial.println("OK rotated on");
      return;
    }
    if (strcmp(value, "off") == 0) {
      gState.rotated = false;
      setRotationEnabled(false);
      Serial.println("OK rotated off");
      return;
    }
    Serial.println("ERR usage: rotated on|off");
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
      Serial.println("ERR usage: save <material>");
      return;
    }
    if (memorySaveMaterial(name, &gConfig, &gState)) {
      Serial.print("OK saved ");
      Serial.println(name);
    } else {
      Serial.println("ERR save failed (bad name or storage error)");
    }
    return;
  }

  if (strcmp(token, "retrieve") == 0 || strcmp(token, "load") == 0) {
    char* name = strtok(nullptr, " \t");
    if (name == nullptr || strtok(nullptr, " \t") != nullptr) {
      Serial.println("ERR usage: retrieve <material>");
      return;
    }
    if (memoryRetrieveMaterial(name, &gConfig, &gState)) {
      // Restored sensor/mode/speed need to reach the chip; rotation and crack
      // settings were already applied inside memoryRetrieveMaterial().
      configureSensor();
      Serial.print("OK retrieved ");
      Serial.println(name);
      printStatus();
    } else {
      Serial.print("ERR no saved material: ");
      Serial.println(name);
    }
    return;
  }

  if (strcmp(token, "materials") == 0) {
    if (strtok(nullptr, " \t") != nullptr) {
      Serial.println("ERR usage: materials");
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
      Serial.println("ERR usage: get_material");
      return;
    }
    memoryMatchByAngle(getRotationAngle());
    return;
  }

  if (strcmp(token, "forget") == 0) {
    char* name = strtok(nullptr, " \t");
    if (name == nullptr || strtok(nullptr, " \t") != nullptr) {
      Serial.println("ERR usage: forget <material>");
      return;
    }
    if (memoryForgetMaterial(name)) {
      Serial.print("OK forgot ");
      Serial.println(name);
    } else {
      Serial.print("ERR no saved material: ");
      Serial.println(name);
    }
    return;
  }

  Serial.print("ERR unknown command: ");
  Serial.println(token);
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

// Snapshots of the live state for main.cpp. Returned by value (small structs)
// so callers can't accidentally mutate our globals through the result.
serial_command_state_t serialCommandsGetState(void) {
  return gState;
}

serial_command_config_t serialCommandsGetConfig(void) {
  return gConfig;
}

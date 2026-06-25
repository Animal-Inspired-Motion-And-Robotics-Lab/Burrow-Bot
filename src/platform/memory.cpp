// Per-material settings persistence on top of the ESP32 Preferences (NVS) API.
//
// Storage model:
//   - Each saved material gets its own NVS namespace, named after the material
//     ("aluminum", "ss", ...). Every user-configurable setting is one key in
//     that namespace. A "v" key holds the schema version and doubles as the
//     "this profile exists" sentinel.
//   - One reserved namespace, "_materials", holds a '\n'-delimited list of the
//     saved material names under the "list" key, so memoryListMaterials() can
//     enumerate them (Preferences/NVS can't list namespaces directly). Material
//     names are forbidden a leading '_' so they can never collide with it.
//
// memory.cpp is the single owner of the NVS key layout: the sensor/mode fields
// come from the serial_commands structs passed in, while the rotation and
// crack-detector settings are read from / written to their owning modules here.
// It deliberately does not touch the LDC1101 — the caller re-pushes config to
// the chip after a retrieve.
//
// NVS (nvs_flash_init) is brought up by the Arduino core at boot, so there's no
// init step; the first Preferences::begin() just works.

#include "memory.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "cmd_reply.h"
#include "crack_detection.h"
#include "measurement_arrays.h"

namespace {

// Reserved index namespace + key. The leading '_' is what we forbid in material
// names, guaranteeing this never aliases a real profile.
constexpr char kIndexNamespace[] = "_materials";
constexpr char kIndexKey[] = "list";

// Working buffer for the '\n'-delimited name index. 16 bytes/name (15 + '\n')
// gives room for ~31 saved materials, well past what the NVS partition holds.
constexpr size_t kIndexBufferLen = 512;

// Bumped if the key set ever changes; written on save, checked on retrieve so a
// half-written or absent namespace reads as "not found". v2 replaced the
// chord-angle phase keys (crk_pmin / crk_pmax) with a single planar-angle key
// (crk_pln) when Stage 2 of the crack detector was reworked. v3 added crk_rpr
// for a short-lived Stage 2b range-ratio cap. v4 replaced crk_rpr with crk_rpd
// when Stage 2b switched to a linear-fit Rp drift cap in ohms. v5 collapsed
// crk_pln + crk_rpd into a single crk_dev (deviation angle of the 3D curve
// from the t-L plane). Older profiles still load — missing keys fall back to
// the live default.
constexpr uint8_t kSchemaVersion = 5;

// Material names map 1:1 to NVS namespace names: 1..MEMORY_MAX_NAME_LEN chars,
// alphanumeric / '-' / '_', and never a leading '_' (reserved for the index).
bool nameValid(const char* name) {
  if (name == nullptr) {return false;}
  size_t n = strlen(name);
  if (n == 0 || n > MEMORY_MAX_NAME_LEN) {return false;}
  if (name[0] == '_') {return false;}
  for (size_t i = 0; i < n; ++i) {
    char c = name[i];
    bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '-' || c == '_';
    if (!ok) {return false;}
  }
  return true;
}

// True if `name` appears as a whole line in the '\n'-delimited `list`. Compares
// full tokens so "alu" doesn't match "aluminum".
bool lineListContains(const char* list, const char* name) {
  size_t nameLen = strlen(name);
  const char* p = list;
  while (*p != '\0') {
    const char* nl = strchr(p, '\n');
    size_t lineLen = nl ? (size_t)(nl - p) : strlen(p);
    if (lineLen == nameLen && strncmp(p, name, nameLen) == 0) {return true;}
    if (nl == nullptr) {break;}
    p = nl + 1;
  }
  return false;
}

// Add `name` to the saved-materials index if it isn't already there.
void indexAdd(const char* name) {
  Preferences idx;
  if (!idx.begin(kIndexNamespace, false)) {return;}

  char buf[kIndexBufferLen];
  buf[0] = '\0';
  idx.getString(kIndexKey, buf, sizeof(buf));

  if (!lineListContains(buf, name)) {
    // Append "name\n" only if it fits; silently skip if the index is full.
    if (strlen(buf) + strlen(name) + 2 <= sizeof(buf)) {
      strcat(buf, name);
      strcat(buf, "\n");
      idx.putString(kIndexKey, buf);
    }
  }
  idx.end();
}

// Remove `name` from the index by rebuilding the list without it.
void indexRemove(const char* name) {
  Preferences idx;
  if (!idx.begin(kIndexNamespace, false)) {return;}

  char buf[kIndexBufferLen];
  buf[0] = '\0';
  idx.getString(kIndexKey, buf, sizeof(buf));

  char out[kIndexBufferLen];
  out[0] = '\0';
  size_t nameLen = strlen(name);
  const char* p = buf;
  while (*p != '\0') {
    const char* nl = strchr(p, '\n');
    size_t lineLen = nl ? (size_t)(nl - p) : strlen(p);
    bool match = (lineLen == nameLen && strncmp(p, name, nameLen) == 0);
    if (!match && lineLen > 0) {
      strncat(out, p, lineLen);
      strcat(out, "\n");
    }
    if (nl == nullptr) {break;}
    p = nl + 1;
  }
  idx.putString(kIndexKey, out);
  idx.end();
}

constexpr float kPi = 3.14159265358979323846f;

// Smallest distance between two trend-line orientations, in [0, π/2]. The
// rotation angle is π-periodic (it's atan of a slope, so θ and θ±π describe the
// same line), so we fold the raw difference into (−π/2, π/2] before taking the
// magnitude. This also makes the near-vertical sign flip — e.g. +89° vs −89°,
// the same near-vertical trend — read as 2° apart rather than 178°.
float orientationDistance(float a, float b) {
  float d = fmodf(a - b, kPi);
  if (d > kPi * 0.5f) {
    d -= kPi;
  } else if (d < -kPi * 0.5f) {
    d += kPi;
  }
  return fabsf(d);
}

}  // namespace

bool memorySaveMaterial(const char* material,
                        const serial_command_config_t* config,
                        const serial_command_state_t* state) {
  if (!nameValid(material) || config == nullptr || state == nullptr) {
    return false;
  }

  Preferences prefs;
  if (!prefs.begin(material, false)) {return false;}

  prefs.putUChar("v", kSchemaVersion);

  // Sensor config (serial_commands).
  prefs.putFloat("l_h", config->sensor_l_h);
  prefs.putFloat("c_f", config->sensor_c_f);
  prefs.putFloat("q", config->sensor_q);
  prefs.putInt("sw_en", config->switch_enable);
  prefs.putInt("sw_gpio", config->switch_gpio);

  // Runtime mode/state (serial_commands).
  prefs.putUChar("mode", (uint8_t)state->mode);
  prefs.putUChar("speed", (uint8_t)state->speed_mode);
  prefs.putBool("stream", state->streaming_enabled);
  prefs.putBool("rotated", state->rotated);
  prefs.putBool("crk_out", state->crack_output);
  prefs.putBool("crk_dbg", state->crack_debug_output);
  prefs.putUInt("delay", state->reading_delay_ms);

  // Signal processing / rotation (measurement_arrays).
  prefs.putUInt("smooth", (uint32_t)getFilterWindow());
  prefs.putFloat("angle", getRotationAngle());
  float centerRp = 0.0f;
  float centerL = 0.0f;
  getRotationCenter(&centerRp, &centerL);
  prefs.putFloat("ctr_rp", centerRp);
  prefs.putFloat("ctr_l", centerL);

  // Crack-detector tuning (crack_detection).
  prefs.putUInt("crk_win", (uint32_t)crackDetectionGetWindowSamples());
  prefs.putFloat("crk_thr", crackDetectionGetThreshold());
  prefs.putFloat("crk_r2", crackDetectionGetMinParabolaR2());
  prefs.putFloat("crk_dev", crackDetectionGetMaxDeviationRad());
  prefs.putFloat("crk_scl", crackDetectionGetLengthEstimateScale());

  prefs.end();

  indexAdd(material);
  return true;
}

bool memoryRetrieveMaterial(const char* material,
                            serial_command_config_t* config,
                            serial_command_state_t* state) {
  if (!nameValid(material) || config == nullptr || state == nullptr) {
    return false;
  }

  // Read-only begin() fails when the namespace was never created, which is our
  // primary "not saved" signal; the version sentinel guards a partial write.
  Preferences prefs;
  if (!prefs.begin(material, true)) {return false;}
  if (prefs.getUChar("v", 0) == 0) {
    prefs.end();
    return false;
  }

  // Sensor config — defaults fall back to the caller's current values so a key
  // missing from an older schema leaves that field untouched.
  config->sensor_l_h = prefs.getFloat("l_h", config->sensor_l_h);
  config->sensor_c_f = prefs.getFloat("c_f", config->sensor_c_f);
  config->sensor_q = prefs.getFloat("q", config->sensor_q);
  config->switch_enable = prefs.getInt("sw_en", config->switch_enable);
  config->switch_gpio = prefs.getInt("sw_gpio", config->switch_gpio);

  // Runtime mode/state.
  state->mode = (ldc1101_mode_t)prefs.getUChar("mode", (uint8_t)state->mode);
  state->speed_mode =
      (ldc_speed_mode_t)prefs.getUChar("speed", (uint8_t)state->speed_mode);
  state->streaming_enabled = prefs.getBool("stream", state->streaming_enabled);
  state->rotated = prefs.getBool("rotated", state->rotated);
  state->crack_output = prefs.getBool("crk_out", state->crack_output);
  state->crack_debug_output = prefs.getBool("crk_dbg", state->crack_debug_output);
  state->reading_delay_ms = prefs.getUInt("delay", state->reading_delay_ms);

  // Signal processing / rotation — applied straight to measurement_arrays.
  setFilterWindow((size_t)prefs.getUInt("smooth", (uint32_t)getFilterWindow()));
  setRotationAngle(prefs.getFloat("angle", getRotationAngle()));
  float centerRp = 0.0f;
  float centerL = 0.0f;
  getRotationCenter(&centerRp, &centerL);
  centerRp = prefs.getFloat("ctr_rp", centerRp);
  centerL = prefs.getFloat("ctr_l", centerL);
  setRotationCenter(centerRp, centerL);
  setRotationEnabled(state->rotated);

  // Crack-detector tuning — applied straight to crack_detection.
  crackDetectionSetWindowSamples(
      (size_t)prefs.getUInt("crk_win", (uint32_t)crackDetectionGetWindowSamples()));
  crackDetectionSetThreshold(prefs.getFloat("crk_thr", crackDetectionGetThreshold()));
  crackDetectionSetMinParabolaR2(
      prefs.getFloat("crk_r2", crackDetectionGetMinParabolaR2()));
  crackDetectionSetMaxDeviationRad(
      prefs.getFloat("crk_dev", crackDetectionGetMaxDeviationRad()));
  crackDetectionSetLengthEstimateScale(
      prefs.getFloat("crk_scl", crackDetectionGetLengthEstimateScale()));

  prefs.end();
  return true;
}

bool memoryForgetMaterial(const char* material) {
  if (!nameValid(material)) {return false;}

  Preferences prefs;
  if (!prefs.begin(material, false)) {return false;}
  bool existed = (prefs.getUChar("v", 0) != 0);
  prefs.clear();  // Drop every key in this namespace.
  prefs.end();

  if (existed) {
    indexRemove(material);
  }
  return existed;
}

void memoryListMaterials(void) {
  Preferences idx;
  // Read-only begin() fails if nothing has ever been saved.
  if (!idx.begin(kIndexNamespace, true)) {
    Reply.println("materials: none");
    return;
  }

  char buf[kIndexBufferLen];
  buf[0] = '\0';
  idx.getString(kIndexKey, buf, sizeof(buf));
  idx.end();

  if (buf[0] == '\0') {
    Reply.println("materials: none");
    return;
  }

  Reply.println("materials:");
  char name[MEMORY_MAX_NAME_LEN + 1];
  const char* p = buf;
  while (*p != '\0') {
    const char* nl = strchr(p, '\n');
    size_t lineLen = nl ? (size_t)(nl - p) : strlen(p);
    if (lineLen > 0 && lineLen <= MEMORY_MAX_NAME_LEN) {
      memcpy(name, p, lineLen);
      name[lineLen] = '\0';

      // Pull the rotation angle out of the material's own namespace.
      float angle = NAN;
      Preferences prefs;
      if (prefs.begin(name, true)) {
        if (prefs.getUChar("v", 0) != 0) {
          angle = prefs.getFloat("angle", NAN);
        }
        prefs.end();
      }

      Reply.print("  ");
      Reply.print(name);
      Reply.print(" angle=");
      Reply.println(angle, 6);
    }
    if (nl == nullptr) {break;}
    p = nl + 1;
  }
}

void memoryMatchByAngle(float target_angle_rad) {
  // Walk the index, opening each saved profile to read its stored "angle", and
  // keep the one whose orientation is closest to the target.
  Preferences idx;
  if (!idx.begin(kIndexNamespace, true)) {
    Reply.println("material: none saved");
    return;
  }

  char buf[kIndexBufferLen];
  buf[0] = '\0';
  idx.getString(kIndexKey, buf, sizeof(buf));
  idx.end();

  if (buf[0] == '\0') {
    Reply.println("material: none saved");
    return;
  }

  char bestName[MEMORY_MAX_NAME_LEN + 1];
  bestName[0] = '\0';
  float bestAngle = NAN;
  float bestDist = NAN;

  char name[MEMORY_MAX_NAME_LEN + 1];
  const char* p = buf;
  while (*p != '\0') {
    const char* nl = strchr(p, '\n');
    size_t lineLen = nl ? (size_t)(nl - p) : strlen(p);
    if (lineLen > 0 && lineLen <= MEMORY_MAX_NAME_LEN) {
      memcpy(name, p, lineLen);
      name[lineLen] = '\0';

      Preferences prefs;
      if (prefs.begin(name, true)) {
        // Skip profiles that don't exist or saved a degenerate (NaN) angle.
        if (prefs.getUChar("v", 0) != 0) {
          float angle = prefs.getFloat("angle", NAN);
          if (!isnan(angle)) {
            float dist = orientationDistance(target_angle_rad, angle);
            if (isnan(bestDist) || dist < bestDist) {
              bestDist = dist;
              bestAngle = angle;
              strcpy(bestName, name);
            }
          }
        }
        prefs.end();
      }
    }
    if (nl == nullptr) {break;}
    p = nl + 1;
  }

  if (bestName[0] == '\0') {
    Reply.println("material: no saved angles to compare");
    return;
  }

  // Report-only: print the match; the operator runs `retrieve` to load it.
  Reply.print("material: closest=");
  Reply.print(bestName);
  Reply.print(" saved_angle=");
  Reply.print(bestAngle, 6);
  Reply.print(" current_angle=");
  Reply.print(target_angle_rad, 6);
  Reply.print(" diff_rad=");
  Reply.print(bestDist, 6);
  Reply.print(" diff_deg=");
  Reply.println(bestDist * (180.0f / kPi), 3);
}

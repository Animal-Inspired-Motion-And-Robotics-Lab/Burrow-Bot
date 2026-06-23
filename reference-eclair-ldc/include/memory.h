// Persistent per-material settings, stored in the ESP32-S3's NVS flash.
//
// A "material profile" is the complete set of user-configurable settings —
// the sensor/mode fields owned by serial_commands plus the rotation and
// crack-detector tuning owned by measurement_arrays / crack_detection. Each
// profile is saved under a material name (e.g. "aluminum") so the operator can
// switch substrates without re-calibrating, and the settings survive a power
// cycle.
//
// Layout (see memory.cpp): one NVS namespace per material (the name itself),
// plus a reserved "_materials" index namespace holding the list of saved names
// so they can be enumerated. NVS namespace names cap at 15 chars, so material
// names are limited to MEMORY_MAX_NAME_LEN.

#ifndef MEMORY_H
#define MEMORY_H

#include <stdbool.h>

#include "serial_commands.h"

// Longest accepted material name. Bounded by the NVS namespace-name limit.
#define MEMORY_MAX_NAME_LEN 15

// Persist the full set of user-configurable settings under `material`.
// Sensor/mode fields are read from *config / *state; the rotation and
// crack-detector settings are read live from their owning modules. Adds the
// name to the saved-materials index. Returns false on an invalid name
// (empty, >MEMORY_MAX_NAME_LEN, leading '_', or non-alphanumeric) or NVS error.
bool memorySaveMaterial(const char* material,
                        const serial_command_config_t* config,
                        const serial_command_state_t* state);

// Load the profile saved under `material`. On success: writes the sensor/mode
// fields back into *config / *state and applies the rotation + crack-detector
// settings directly to their modules (including rotation-enable). The caller
// must still push *config to the chip (configureSensor()) — memory.cpp does not
// touch the LDC1101. Returns false if no profile was saved under that name.
bool memoryRetrieveMaterial(const char* material,
                            serial_command_config_t* config,
                            serial_command_state_t* state);

// Delete a saved profile and remove it from the index. Returns false if the
// name is invalid or no profile existed under it.
bool memoryForgetMaterial(const char* material);

// Print the saved materials over Serial, one per line as "<name> angle=<rad>"
// (the stored rotation angle), or a "none" notice when the index is empty.
void memoryListMaterials(void);

// Identify which saved profile best matches `target_angle_rad` (typically the
// just-calibrated rotation angle from getRotationAngle()) and print the match
// over Serial, along with the saved/current angles and their difference.
// Matching uses a π-periodic orientation distance, since the rotation angle is
// a line orientation (it comes from atan of a slope, so θ and θ±π are the same
// trend). Report-only: it changes nothing — run `retrieve <name>` to load the
// match. Prints a notice when no profiles (or no saved angles) exist.
void memoryMatchByAngle(float target_angle_rad);

#endif

// Line-oriented serial CLI for runtime configuration. The CLI owns two
// structs that the main loop reads each tick: `config` for fields used to
// configure the LDC1101, and `state` for the modes that just affect the
// per-tick read/print loop. See serial_commands.cpp for the dispatch layout
// and the help/status mirror convention.

#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include <stdbool.h>

#include "ldc1101.h"

// Register-affecting sensor configuration. Stored in SI base units.
typedef struct {
  float sensor_l_h;     // Sensor inductance (H). CLI takes µH.
  float sensor_c_f;     // Sensor capacitance (F). CLI takes pF.
  float sensor_q;       // Quality factor of the LC tank.
  int switch_enable;    // 1 = drive an external mux/switch before configuring.
  int switch_gpio;      // GPIO number for that switch (-1 = unused).
} serial_command_config_t;

// Runtime mode flags read by main.cpp each tick. Changing these doesn't
// require reconfiguring the LDC1101 (mode and speed_mode are exceptions and
// trigger a reconfigure inside their own command handlers).
typedef struct {
  ldc1101_mode_t mode;          // RP+L vs LHR
  ldc_speed_mode_t speed_mode;  // Accuracy↔throughput tradeoff.
  bool streaming_enabled;       // Master streaming on/off.
  bool rotated;                 // Mirror of measurement_arrays' rotationEnabled.
  bool crack_output;            // Emit per-crack Teleplot fields on detection.
  bool crack_debug_output;      // Emit reject_reason and debug key=value lines.
  uint32_t reading_delay_ms;    // Sample period.
} serial_command_state_t;

// Adopt the seeds, push them to the LDC1101, print help. Call once at boot.
void serialCommandsInit(const serial_command_config_t* config,
                        const serial_command_state_t* initial_state);

// Drain pending Serial input and dispatch any complete '\n'-terminated lines.
// Non-blocking; safe to call every tick.
void serialCommandsPoll(void);

// Snapshots of the live state/config for the main loop.
serial_command_state_t serialCommandsGetState(void);
serial_command_config_t serialCommandsGetConfig(void);

#endif

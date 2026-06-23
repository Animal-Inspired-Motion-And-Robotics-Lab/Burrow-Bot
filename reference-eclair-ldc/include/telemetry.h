// Streaming serial output for the measurement pipeline. Owns the Teleplot
// wire format so changes to the per-tick output live in one place. See
// telemetry.cpp for the layout of the stream / detection / debug categories.

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#include "crack_detection.h"
#include "serial_commands.h"

// Emit one tick's serial output. Always writes the >Rp/>L/>t line; optionally
// appends per-detection fields (when state->crack_output) and a >reason +
// debug line (when state->crack_debug_output).
void telemetryEmitSample(uint32_t now_ms,
                         const serial_command_state_t* state,
                         bool crack_detected,
                         const crack_detection_result_t* crack);

#endif
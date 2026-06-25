# Burrow-Bot (eddy-burrow-bot)

Mobile eddy-current crack-detection robot. A joystick-driven robot carries a TI
**LDC1101** inductance-to-digital sensor over a surface; the on-device firmware
runs a crack-detection pipeline and streams measurements back to a PC that
plots, labels, and records them (with synchronized video).

This repo is the **eddy-burrow-bot** branch: the burrow-bot hardware running the
crack-detection demo originally built on the separate **ECLAIR** robot. The
ECLAIR pipeline was migrated here in 2026-06 (see "LDC pipeline integration").

## System architecture (three pieces, one PlatformIO project)

```
 PC (pc_control, Python)  <--USB-->  Bridge ESP32  <--UART-->  Platform ESP32
   joystick + plotting +              transparent              motors + LDC1101 +
   CSV recording                      byte forwarder           crack pipeline
```

- **PC** — `pc_control/BBot LDC Scanner.py` (the active GUI; `AAA Opt BBot
  Control Code.py` is the older, still-request/response variant). Pygame
  joystick → motor commands at 100 Hz; pyqtgraph plots (3D Rp/L/t surface +
  phase-space scatter); records to CSV. **Free-running, not request/response:**
  motor/CLI commands go out fire-and-forget and telemetry is drained
  continuously and non-blocking — no per-sample handshake. (The old
  send-one-wait-for-one model serialized every sample behind a USB round-trip
  and was the source of the control/telemetry lag.)
- **Bridge** — `src/bridge/main.cpp`. A XIAO ESP32-S3 that forwards every byte
  USB↔UART in both directions, unframed. **Protocol-agnostic — never needs to
  change when the packet format changes.**
- **Platform** — `src/platform/main.cpp` + the pipeline modules. Free-running:
  `loop()` reads the LDC1101, runs the crack pipeline, and streams a telemetry
  packet every iteration, while `serviceSerial1()` applies streamed motor
  commands and dispatches CLI frames asynchronously (non-blocking).

## Build & flash (PlatformIO)

PlatformIO Core lives at `~/.platformio/penv/Scripts/pio.exe` (not on PATH).

```bash
PIO="$HOME/.platformio/penv/Scripts/pio.exe"
"$PIO" run -e platform              # build Platform firmware (default env)
"$PIO" run -e bridge                # build bridge firmware
"$PIO" run -e platform -t upload    # flash Platform
"$PIO" run -e bridge   -t upload    # flash bridge
```

Both boards are `seeed_xiao_esp32s3`, Arduino framework. `build_src_filter`
splits the two firmwares: `+<platform/*>` vs `+<bridge/*>`, so the pipeline
sources under `src/platform/` only compile into the Platform build.

`pc_control` deps are in `requirements.txt` (pyserial, pygame, opencv, numpy,
pyqtgraph). Experiment data records to CSV via the GUI's "Write to File" toggle
(also bound to the controller A button); HDF5/video recording was removed.

## Telemetry packet (Platform → PC, little-endian, 27 bytes)

Defined in `src/platform/main.cpp` (`sendLDCResponse`) and parsed in the Python
app (`SerialComm.drain_packets` / `_consume_frames`, `RESP_LEN = 27`). Keep in sync.

| Offset | Type   | Field        | Notes                                        |
|--------|--------|--------------|----------------------------------------------|
| 0      | u8     | START 0xAA   |                                              |
| 1..8   | u64    | timestamp    | `micros()`                                   |
| 9..10  | i16    | motor L PWM  |                                              |
| 11..12 | i16    | motor R PWM  |                                              |
| 13..16 | f32    | Rp (ohms)    | raw; rotated into the calibrated frame if bit1 |
| 17..20 | f32    | L (uH)       | raw; rotated into the calibrated frame if bit1 |
| 21     | u8     | flags        | bit0 = crack detected, bit1 = rotated/calibrated |
| 22..25 | f32    | crack_size   | thou (fit peak height × length scale)        |
| 26     | u8     | END 0x55     |                                              |

Motor command (PC → Platform): 6 bytes
`[0xAA][L i16 big-endian][R i16 big-endian][0x55]`.

CLI command frame (PC → Platform): `[0xAB][len u8][len ASCII bytes][0x55]`. The
Platform parses `0xAB` frames in `serviceSerial1()`'s state machine and feeds the
text to `serialCommandsProcessLine()` — the **same dispatcher as the USB CLI**, so
any CLI command works (`calibrate`, `rotated on|off`, `save <name>`, `crack_*`, ...).
Command frames get no telemetry reply (the reply text streams back as 0xAC frames;
see below). PC side: `SerialComm.send_command(text)`; joystick buttons enqueue into
the shared `command_queue`, drained each control tick.

CLI reply frame (Platform → PC): `[0xAC][len u8][len ASCII bytes][0x55]`, one per
output line. The CLI dispatcher and its helpers (`serial_commands`,
`calibration`, `memory`) print through the `Reply` tee (`cmd_reply.h/.cpp`)
instead of `Serial` directly: it always mirrors to the USB-CDC `Serial`, and
while `serialCommandsProcessLine()` brackets a bridge-sourced command with
`Reply.beginBridgeCapture()/endBridgeCapture()` it also frames each line onto
`Serial1`. (Teleplot in `telemetry.cpp` still writes `Serial` directly — only
command output is framed, not the high-rate stream.) The PC demuxes `0xAA`
telemetry and `0xAC` reply frames from the one stream in
`SerialComm._consume_frames()`; reply lines queue onto `SerialComm.responses`,
drained each `control_tick` into the GUI's Platform-CLI transcript. Replies are
**not** synchronous — they appear a tick or two after the command. Controller
binds (PC `JoystickControl.handle_button`): **X = `calibrate`, Y = toggle
`rotated`, A = CSV write, B = material marker, L-click = reset plots, R-click =
`vibrate`** (rotate toggle synced from telemetry flag bit1). `calibrate`/`baseline`
no longer block (the `ledFlash` was removed) so they're safe to issue mid-`vibrate`;
a ~600 ms block there would freeze the vibrate mid-phase and drive the robot.

To add a telemetry field: append before END, bump `RESP_LEN` and the firmware
buffer size + END index together.

## On-device LDC pipeline (`src/platform/`, ported from ECLAIR)

Per-loop chain (free-running — the Platform streams one sample per tick, paced by
the CLI `delay` / `reading_delay_ms`, default 25 ms = 40 Hz; the PC drains
continuously):

```
ldc1101_read()        raw (Rp, L)
   → appendMeasurement()    smooth (moving avg) + rotate + store    (measurement_arrays)
   → crackDetectionCheck()  parabola fit + planarity + dedup        (crack_detection)
   → flags/crack_size in the reply packet (+ LED, + optional Teleplot)
```

Modules (headers in `include/`, sources in `src/platform/`):
- `measurement_arrays` — smoothing window, rotation (flatten the (Rp,L) cloud), history.
- `crack_detection` — multi-stage detector over the rotated stream.
- `calibration` — one-shot per-substrate rotation (`calibrate`) + baseline re-zero.
- `memory` — per-material profiles in NVS (`save`/`retrieve`/`materials`).
- `serial_commands` — line CLI owning the live config/state structs.
- `telemetry` — Teleplot stream (USB only; off by default).
- `LED` — polarity-aware status LED helper.

### Driver — `lib/LDC1101/`
`include/ldc1101.h` + `src/ldc1101.cpp`. **ESP-IDF** SPI (`driver/spi_master.h`,
FreeRTOS), so it owns a whole SPI host via `spi_bus_initialize`. 7-arg
`ldc1101_configure(L, C, Q, mode, speed, switch_en, switch_gpio)` with RP+L and
LHR modes and four speed/accuracy presets; plus `sleep()`/`wake()`. Pins and SPI
host come from `-D` build flags in `platformio.ini`:
`LDC_MOSI=9 LDC_MISO=8 LDC_SCK=7 LDC_CS=6 LDC_SPI_HOST=SPI3_HOST LDC_SPI_CLOCK_HZ=100000`.

## Two serial ports (the key integration decision)

The Platform uses **both** UARTs, which sidesteps the ECLAIR demo's serial
conflict entirely:
- **`Serial1`** (pins RX=44 / TX=43) — the binary motor/telemetry protocol to
  the PC via the bridge. The robot's real-time control link.
- **`Serial`** (USB-CDC) — the ECLAIR CLI: tuning, `calibrate`, material memory,
  and optional Teleplot. Bench/tethered use; idle while driving untethered.

## Calibration workflow (operator, over USB)

Cracks are only flagged **after calibration** (flags bit1). The pipeline now
free-runs (advances every Platform loop regardless of PC traffic), so calibration
only needs the sensor over the substrate — connect USB to the Platform, then over
the USB serial CLI: `calibrate` → optionally `save <material>` / `retrieve
<material>` / `materials`. `help` / `status` list everything.

**Vibrate (experimental liftoff dither).** `vibrate [strength freq_hz duration_ms]`
runs a non-blocking burst that alternately twists the treads in opposite
directions (L+,R- ↔ L-,R+) to jostle the sensor and vary liftoff *while telemetry
keeps streaming* — the goal being to let `calibrate` separate material
conductivity from liftoff. Bare `vibrate` re-runs with the last params; the
right-stick click sends it. The pulsing lives in `main.cpp` (`startVibrate` +
loop, which owns the motors and overrides PC motor commands during a burst);
`serial_commands` stages the params + a one-shot flag
(`serialCommandsTakeVibrateRequest`). Defaults: 180 PWM / 20 Hz / 1500 ms — tune
to find what mechanically resonates this chassis. (Rapid full-PWM reversals
stress the DRV8835/motors; keep bursts short while experimenting.)

## Pin map (Platform, XIAO ESP32-S3)

| Pins        | Use                                          |
|-------------|----------------------------------------------|
| 1,2,3,4     | DRV8835 motors AIN1/AIN2/BIN1/BIN2 (PWM)     |
| 7,8,9,6     | LDC1101 SCK / MISO / MOSI / CS (SPI3_HOST)   |
| 43,44       | Serial1 TX/RX (UART to bridge)               |
| 21          | status LED (active-high)                     |

The original code also init'd I2C on SDA=5/SCL=6 for an (unused) IMU — removed
during integration because SCL=6 collided with LDC CS=6. **Verify CS=6 matches
the actual Platform wiring before flashing.**

## Gotchas

- This ESP32 core uses the **per-pin** `analogWriteFrequency(pin, freq)` /
  `analogWriteResolution(pin, bits)` API, not the old global form.
- USB-CDC `Serial` can stall if it writes faster than a host reads, so Teleplot
  (`streaming_enabled`) defaults **off**; enable with `stream on` when tethered.
- Crack indication is a non-blocking LED latch (not the ECLAIR blocking
  `ledFlash`), to keep the free-running loop streaming telemetry at full rate.
- `reference-eclair-ldc/` (with `INTEGRATION.md`) and `lib-updated-ldc1101/` are
  source-of-truth scaffolding the integration was copied from; not part of the
  build.

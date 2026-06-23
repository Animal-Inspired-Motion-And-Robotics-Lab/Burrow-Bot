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
 PC (pc_control, Python)  <--USB-->  Bridge ESP32  <--UART-->  Platform ESP32 (antenna)
   joystick + plotting +              transparent              motors + LDC1101 +
   HDF5/video recording               byte forwarder           crack pipeline
```

- **PC** — `pc_control/AAA Opt BBot Control Code.py`. Pygame joystick → motor
  commands at 100 Hz; reads back telemetry; pyqtgraph plots (time series + Rp/L
  phase-space scatter); records HDF5 + camera video. Request/response: send one
  motor packet, wait for one telemetry packet.
- **Bridge** — `src/bridge/main.cpp`. A XIAO ESP32-S3 that forwards every byte
  USB↔UART in both directions, unframed. **Protocol-agnostic — never needs to
  change when the packet format changes.**
- **Platform / antenna** — `src/platform/main.cpp` + the pipeline modules. The
  responder: waits for a motor command, drives the DRV8835 motors, reads the
  LDC1101, runs the crack pipeline, replies with a telemetry packet.

## Build & flash (PlatformIO)

PlatformIO Core lives at `~/.platformio/penv/Scripts/pio.exe` (not on PATH).

```bash
PIO="$HOME/.platformio/penv/Scripts/pio.exe"
"$PIO" run -e platform              # build antenna firmware (default env)
"$PIO" run -e bridge                # build bridge firmware
"$PIO" run -e platform -t upload    # flash antenna
"$PIO" run -e bridge   -t upload    # flash bridge
```

Both boards are `seeed_xiao_esp32s3`, Arduino framework. `build_src_filter`
splits the two firmwares: `+<platform/*>` vs `+<bridge/*>`, so the pipeline
sources under `src/platform/` only compile into the antenna build.

`pc_control` deps are in `requirements.txt` (pyserial, pygame, opencv, h5py,
numpy, pyqtgraph).

## Telemetry packet (antenna → PC, little-endian, 27 bytes)

Defined in `src/platform/main.cpp` (`sendLDCResponse`) and parsed in the Python
app (`SerialComm.read_ldc_packet`, `RESP_LEN = 27`). Keep the two in sync.

| Offset | Type   | Field        | Notes                                        |
|--------|--------|--------------|----------------------------------------------|
| 0      | u8     | START 0xAA   |                                              |
| 1..8   | u64    | timestamp    | `micros()`                                   |
| 9..10  | i16    | motor L PWM  |                                              |
| 11..12 | i16    | motor R PWM  |                                              |
| 13..16 | f32    | Rp (ohms)    | raw, pre-rotation                            |
| 17..20 | f32    | L (uH)       | raw, pre-rotation                            |
| 21     | u8     | flags        | bit0 = crack detected, bit1 = calibrated     |
| 22..25 | f32    | crack_size   | thou (fit peak height × length scale)        |
| 26     | u8     | END 0x55     |                                              |

Motor command (PC → antenna): 6 bytes
`[0xAA][L i16 big-endian][R i16 big-endian][0x55]`.

CLI command frame (PC → antenna): `[0xAB][len u8][len ASCII bytes][0x55]`. The
antenna recognizes `0xAB` in `readMotorCommand()`'s wait loop and feeds the text
to `serialCommandsProcessLine()` — the **same dispatcher as the USB CLI**, so any
CLI command works (`calibrate`, `rotated on|off`, `save <name>`, `crack_*`, ...).
Command frames get no telemetry reply. PC side: `SerialComm.send_command(text)`;
joystick buttons enqueue into `JoystickControl.pending_commands`, drained each
loop. Stick-clicks are bound: **L-click = `calibrate`, R-click = toggle
`rotated`** (toggle state synced from telemetry flag bit1). Note `calibrate`
blocks the antenna ~600 ms (its `ledFlash`), so do it stationary.

To add a telemetry field: append before END, bump `RESP_LEN` and the firmware
buffer size + END index together.

## On-device LDC pipeline (`src/platform/`, ported from ECLAIR)

Per-tick chain, one sample per PC request (PC paces the ~100 Hz cadence):

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

The antenna uses **both** UARTs, which sidesteps the ECLAIR demo's serial
conflict entirely:
- **`Serial1`** (pins RX=44 / TX=43) — the binary motor/telemetry protocol to
  the PC via the bridge. The robot's real-time control link.
- **`Serial`** (USB-CDC) — the ECLAIR CLI: tuning, `calibrate`, material memory,
  and optional Teleplot. Bench/tethered use; idle while driving untethered.

## Calibration workflow (operator, over USB)

Cracks are only flagged **after calibration** (flags bit1). To calibrate a
substrate: connect USB to the antenna, ensure the PC control app is running
(data must be flowing — the pipeline only advances on motor requests), then over
the USB serial CLI: `calibrate` → optionally `save <material>` / `retrieve
<material>` / `materials`. `help` / `status` list everything.

## Pin map (antenna, XIAO ESP32-S3)

| Pins        | Use                                          |
|-------------|----------------------------------------------|
| 1,2,3,4     | DRV8835 motors AIN1/AIN2/BIN1/BIN2 (PWM)     |
| 7,8,9,6     | LDC1101 SCK / MISO / MOSI / CS (SPI3_HOST)   |
| 43,44       | Serial1 TX/RX (UART to bridge)               |
| 21          | status LED (active-high)                     |

The original code also init'd I2C on SDA=5/SCL=6 for an (unused) IMU — removed
during integration because SCL=6 collided with LDC CS=6. **Verify CS=6 matches
the actual antenna wiring before flashing.**

## Gotchas

- This ESP32 core uses the **per-pin** `analogWriteFrequency(pin, freq)` /
  `analogWriteResolution(pin, bits)` API, not the old global form.
- USB-CDC `Serial` can stall if it writes faster than a host reads, so Teleplot
  (`streaming_enabled`) defaults **off**; enable with `stream on` when tethered.
- Crack indication is a non-blocking LED latch (not the ECLAIR blocking
  `ledFlash`), to keep the 100 Hz response loop from missing PC requests.
- `reference-eclair-ldc/` (with `INTEGRATION.md`) and `lib-updated-ldc1101/` are
  source-of-truth scaffolding the integration was copied from; not part of the
  build.

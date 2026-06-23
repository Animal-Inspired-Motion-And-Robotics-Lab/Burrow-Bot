# Integrating this LDC1101 / crack-detection firmware into another robot

This repo is the sensor-electronics firmware for a TI **LDC1101** eddy-current crack
detector (see [CLAUDE.md](CLAUDE.md) for the full architecture). It was built to run
standalone on a **Seeed XIAO ESP32-S3** under the PlatformIO + Arduino framework.

**Confirmed target (2026-06):** the other robot is **ESP32 + PlatformIO** — the cleanest
case, so the driver works as-is and integration is mostly merging `build_flags` and folding
`setup()`/`loop()` (steps 2 + 4). Whether the host firmware already uses SPI / `Serial` /
NVS was still TBD, so treat steps 3, 5, and 6 as "verify, don't assume safe."

This document is the drop-in guide for merging it into a **different robot's firmware**.
Read it before combining the two `main`s. It exists because the most useful integration
context cannot live in a Claude `~/.claude` memory file — that memory is keyed to a
specific project path and does **not** travel when you copy these files into another
repo. This file does travel. When you ask Claude to "combine the two firmwares" in the
other repo, point it here first.

---

## TL;DR — what's portable and what fights

| Layer | Files | Portable? | Integration risk |
|---|---|---|---|
| **LDC1101 driver** | `lib/LDC1101/` | Yes, clean C API | **ESP32-only** (ESP-IDF SPI/GPIO); owns its SPI bus |
| **Measurement + detector** | `src/measurement_arrays.cpp`, `crack_detection.cpp`, `calibration.cpp`, `telemetry.cpp` | Yes, plain C++ | uses Arduino `Serial`/`millis` (telemetry only) |
| **Serial CLI** | `src/serial_commands.cpp` | Yes, but… | **owns the `Serial` port and the input line discipline** |
| **Persistence** | `src/memory.cpp` | Yes | uses Arduino `Preferences` (NVS) — **namespace collisions** |
| **LED feedback** | `src/LED.cpp` | Yes | uses `LED_BUILTIN` |
| **Orchestration glue** | `src/main.cpp` | **No — rewrite/merge** | this is where `setup()`/`loop()` collide |

The driver + the four pipeline modules are the prize and are cleanly reusable.
`main.cpp` is glue, not a deliverable — you will fold its `setup()` body and its
`loop()` body into the host firmware rather than copy it.

---

## Hard prerequisites for the target platform

1. **Must be an ESP32.** `lib/LDC1101/src/ldc1101.cpp` is written against ESP-IDF
   (`driver/spi_master.h`, `driver/gpio.h`, FreeRTOS). It compiles only because the
   Arduino-ESP32 core re-exports ESP-IDF. It will **not** build on AVR/SAMD/RP2040/STM32.
   If the other robot is not ESP32-based, the driver has to be reimplemented against
   that MCU's SPI — the application layer above it is portable, the driver is not.
2. **Arduino API must be available** for the application layer (`Serial`, `millis`,
   `delay`, `Preferences`, `LED_BUILTIN`). Either Arduino framework, or ESP-IDF with
   Arduino-as-a-component.

---

## Step-by-step merge

### 1. Files to copy
Copy `lib/LDC1101/` and the application sources/headers (`src/*.cpp` except `main.cpp`,
plus `include/*.h`). **Do not** copy `main.cpp` wholesale — see step 4.

### 2. Build flags (the LDC pins live here, not in source)
The driver's pins and SPI host come from `-D` defines, with `#ifndef` fallbacks in
`ldc1101.cpp`. Current values from this repo's `platformio.ini`:

```ini
build_flags =
    -D LDC_MOSI=9
    -D LDC_MISO=8
    -D LDC_SCK=7
    -D LDC_CS=4
    -D LDC_SPI_HOST=SPI3_HOST
    -D LDC_SPI_CLOCK_HZ=100000
```

Merge these into the host project's `build_flags`. **Re-pin them to free GPIOs on the
new robot** and pick an SPI host the host firmware isn't already using (see step 3).
Also ensure the host build adds `include/` and `lib/LDC1101/include/` to its include
path — PlatformIO does this automatically; a hand-rolled build/Arduino-IDE layout does not.

### 3. SPI bus ownership — the biggest hardware gotcha
`ldc1101_init()` calls `spi_bus_initialize(LDC_SPI_HOST, …)` ([ldc1101.cpp:283](lib/LDC1101/src/ldc1101.cpp#L283)),
i.e. it **takes exclusive ownership of that whole SPI host**. If the other robot already
uses the same host (e.g. for an IMU, SD card, or display), `spi_bus_initialize` will fail
with `ESP_ERR_INVALID_STATE` and `ESP_ERROR_CHECK` will abort/reboot. Options:
   - **Easiest:** give the LDC1101 its own free SPI host and pins (re-pin in step 2).
   - **Shared bus:** the host firmware owns `spi_bus_initialize`, and the driver is
     refactored to only `spi_bus_add_device` onto the existing bus. This is a driver
     edit — flag it when it comes up.
   Also confirm the four LDC pins don't collide with anything on the new robot.

### 4. Merge `setup()` and `loop()` (do not keep a second `setup`/`loop`)
The Arduino core allows exactly one `setup()`/`loop()`. Fold this firmware's logic into
the host's:

   - **setup():** run the boot sequence from [src/main.cpp:54-97](src/main.cpp#L54-L97) —
     `ldc1101_init()` → `serialCommandsInit(&commandConfig, &initialState)` →
     `setFilterWindow(...)` → `crackDetectionInit(&crackConfig)`. Drop the
     `Serial.begin(9600)` / `delay(5000)` / LED-flash bits if the host already owns
     boot/serial. Keep the boot-default sensor params (`kSensorL_H`, `kSensorC_F`,
     `kSensorQ`) — they're just seeds; live values are owned by `serial_commands`.
   - **loop():** call the per-tick pipeline from [src/main.cpp:100-135](src/main.cpp#L100-L135)
     as a subroutine of the host loop: `serialCommandsPoll()` → snapshot state/config →
     respect the non-blocking `reading_delay_ms` pacing → `ldc1101_read` →
     `appendMeasurement` → `crackDetectionCheck` → `telemetryEmitSample`. Keep it
     **non-blocking** (no `delay()` in the merged loop) so the host robot's control loop
     keeps running.

### 5. Serial port arbitration
`serial_commands.cpp` runs a **line-oriented CLI on `Serial`** and `telemetry.cpp`
streams Teleplot lines on the same port. If the host robot also uses `Serial` for its own
protocol/telemetry, they **will interleave and corrupt each other**. Decide one of:
   - Put the LDC CLI + Teleplot stream on a **separate UART** (`Serial1`/`Serial2`) — cleanest.
   - Multiplex: route the host's command parser to call `processCommand()` for LDC
     commands and gate Teleplot output behind a mode flag.
   - Accept shared serial only if the host has no competing serial traffic.

### 6. NVS namespace collisions
`memory.cpp` stores material profiles via Arduino `Preferences` (NVS): one namespace
**per material name**, plus a reserved `_materials` index namespace. If the host firmware
also uses NVS, ensure its namespaces never equal a material name or `_materials`. Names
are capped at `MEMORY_MAX_NAME_LEN` = 15 (the NVS limit) and leading `_` is reserved.
Also confirm both firmwares agree on the NVS partition.

### 7. Symbol / resource collisions to scan for
   - `setup`, `loop`, `fw_version`, `LED_BUILTIN` usage.
   - The LED: `LED.cpp` assumes the XIAO's active-**low** built-in LED
     (`kLedActiveHigh = false`). Re-point/re-polarize for the new board, or skip
     `ledInit`/`ledFlash` if the host owns the LED.
   - Any duplicate global names between the two codebases.

---

## Verification checklist after merging
- [ ] `pio run` (or host build) is clean — no duplicate-symbol / missing-header errors.
- [ ] LDC SPI host + 4 pins are exclusive to the LDC1101 (no peripheral conflict).
- [ ] `ldc1101_init()` does not abort at boot (SPI bus init succeeded).
- [ ] Boot banner + `status` over the chosen serial port responds to CLI commands.
- [ ] `calibrate` then a swipe produces a Teleplot stream; a crack swipe flashes/reports.
- [ ] `save`/`retrieve`/`materials` round-trip without clobbering host NVS data.
- [ ] Host robot's own control loop still meets timing (LDC loop stays non-blocking).

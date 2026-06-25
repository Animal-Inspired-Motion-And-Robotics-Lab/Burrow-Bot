//  _____  _       _    __
// |  __ \| |     | |  / _|
// | |__) | | __ _| |_| |_ ___  _ __ _ __ ___
// |  ___/| |/ _` | __|  _/ _ \| '__| '_ ` _ \
// | |    | | (_| | |_| || (_) | |  | | | | | |
// |_|    |_|\__,_|\__|_| \___/|_|  |_| |_| |_|
//
// ===== PLATFORM - RESPONDER MODE + ON-DEVICE LDC PIPELINE =====
// Request/response over Serial1 (binary protocol to the PC via the bridge):
// waits for a motor command, drives the motors, reads the LDC1101, runs the
// ECLAIR crack-detection pipeline, and replies with an extended telemetry
// packet.
//
// Two serial ports, two jobs:
//   * Serial1 (UART, pins 43/44) -> binary motor/telemetry protocol to the PC.
//   * Serial  (USB-CDC)          -> ECLAIR-style CLI for live tuning,
//                                    calibration, and material memory, plus an
//                                    optional Teleplot debug stream. Bench use
//                                    only; idle during untethered driving.
//
// Per-tick pipeline (mirrors reference-eclair-ldc/src/main.cpp):
//   ldc1101_read() -> appendMeasurement() -> crackDetectionCheck() -> reply

#include <Arduino.h>

#include "ldc1101.h"
#include "LED.h"
#include "crack_detection.h"
#include "measurement_arrays.h"
#include "serial_commands.h"
#include "telemetry.h"

// fw_version is referenced by the CLI/telemetry banners.
const char* fw_version = "0.3.0-burrow";

// ===== CONFIGURATION =====
#define BAUD_RATE 460800

// Packet markers
#define PKT_START 0xAA  // motor command frame (PC->Platform) and telemetry frame (Platform->PC)
#define CMD_START 0xAB  // ASCII CLI command frame (PC->Platform): [0xAB][len][ascii][0x55]
#define PKT_END 0x55

// Pins
#define UART_TX_PIN 43
#define UART_RX_PIN 44

// Motor pins (DRV8835 IN/IN mode)
#define AIN1 1
#define AIN2 2
#define BIN1 3
#define BIN2 4

// Platform status LED (active-high on this board).
#define LED_PIN 21

// ===== LDC PIPELINE BOOT DEFAULTS =====
// All of these are reconfigurable at runtime via the USB CLI; they are just the
// power-on seeds. Values match the burrow-bot's stacked inductor (L = 11.8 uH,
// the smallest of the 11.8 / 42.6 / 90.0 uH stack) and 220 pF tank.
static constexpr float kSensorL_H = 11.8e-6f;  // H
static constexpr float kSensorC_F = 220e-12f;  // F
static constexpr float kSensorQ = 15.0f;       // LC tank quality factor

// External mux/switch driven by the driver before configuring. Unused here.
static constexpr int kSwitchEnable = 0;
static constexpr int kSwitchGpio = -1;

// The loop free-runs, so the telemetry stream rate is paced by the CLI `delay`
// (reading_delay_ms): the sensor is read + a packet streamed at most once every
// reading_delay_ms (1000/delay Hz). Boot default 25 ms = 40 Hz; change live with
// `delay <ms>`. A small value approaches the LDC read rate (the hard ceiling).
static constexpr uint32_t kDefaultReadingDelayMs = 25;

// Onboard LED polarity (XIAO active-low; this Platform LED on pin 21 is active-high).
static constexpr bool kLedActiveHigh = true;

// Moving-average smoothing window applied before rotation/storage (1 disables).
static constexpr size_t kFilterWindow = 25;

// ===== MOTOR STATE =====
int16_t currentLeftPWM = 0;
int16_t currentRightPWM = 0;

// ===== PACKET BUFFERS =====
// Response layout (little-endian, 27 bytes):
//   [0]      0xAA  START
//   [1..8]   uint64  timestamp (micros)
//   [9..10]  int16   motor L PWM
//   [11..12] int16   motor R PWM
//   [13..16] float   Rp (ohms)         raw, or rotated into the calibrated frame
//   [17..20] float   L  (uH)           when `rotated` is on (flags bit1)
//   [21]     uint8   flags  bit0=crack detected, bit1=rotated/calibrated
//   [22..25] float   crack_size (thou) fit peak height * length-estimate scale
//   [26]     0x55  END
static uint8_t txBuffer[27];

// Non-blocking crack indicator: latch the LED on for a short window instead of
// the ECLAIR firmware's blocking ledFlash(), which would stall the ~100 Hz
// request/response loop and trip PC-side response timeouts.
static uint32_t crackLedOffMs = 0;
static constexpr uint32_t kCrackLedHoldMs = 60;

// Pacing for the telemetry stream: timestamp (millis) of the last sampled tick.
// The loop reads the sensor + streams a packet only once reading_delay_ms has
// elapsed since this (see loop()); inbound motor/CLI handling stays every-loop.
static uint32_t lastSampleMs = 0;

// ===== MOTOR FUNCTIONS =====
void setLeftMotor(int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(AIN1, pwm);
    analogWrite(AIN2, 0);
  } else if (pwm < 0) {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, -pwm);
  } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 0);
  }
}

void setRightMotor(int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(BIN1, pwm);
    analogWrite(BIN2, 0);
  } else if (pwm < 0) {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, -pwm);
  } else {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 0);
  }
}

void setMotors(int16_t left, int16_t right) {
  setLeftMotor(left);
  setRightMotor(right);
  currentLeftPWM = left;
  currentRightPWM = right;
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  currentLeftPWM = 0;
  currentRightPWM = 0;
}

// ===== SETUP =====
void setup() {
  // UART link to the bridge/PC (binary protocol). Short read timeout so a torn
  // command frame can't block the response loop in readBytes().
  Serial1.begin(BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial1.setTimeout(20);

  // USB-CDC: ECLAIR CLI + optional Teleplot. USB-CDC ignores the baud value.
  Serial.begin(115200);

  // Motor pins. This ESP32 core uses the per-pin analogWrite* API, so configure
  // 8-bit / 1 kHz PWM on each motor pin individually.
  const int motorPins[4] = {AIN1, AIN2, BIN1, BIN2};
  for (int pin : motorPins) {
    pinMode(pin, OUTPUT);
    analogWriteResolution(pin, 8);
    analogWriteFrequency(pin, 1000);
  }
  setMotors(0, 0);

  // Status LED.
  ledInit(LED_PIN, kLedActiveHigh);

  // ----- LDC pipeline boot (mirrors reference-eclair-ldc/src/main.cpp) -----
  // Seed the CLI; after this every CLI command mutates these in place.
  serial_command_config_t commandConfig = {kSensorL_H, kSensorC_F, kSensorQ,
                                           kSwitchEnable, kSwitchGpio};
  // {mode, speed, streaming_enabled, rotated, crack_output, crack_debug, delay}
  // streaming starts OFF so the robot doesn't push Teleplot text into an
  // unread USB-CDC buffer while driving untethered; enable with `stream on`.
  serial_command_state_t initialState = {LDC1101_MODE_RP_L, LDC_SPEED_BALANCED_1,
                                         false, false, true, false,
                                         kDefaultReadingDelayMs};

  // ldc1101_init() prepares the SPI bus; serialCommandsInit() then pushes the
  // seeded sensor/mode/speed values to the chip via ldc1101_configure().
  ldc1101_init();
  serialCommandsInit(&commandConfig, &initialState);

  setFilterWindow(kFilterWindow);

  // Crack detector tuning — boot defaults, retunable live via `crack_*`.
  crack_detection_config_t crackConfig = {
      0.01f,    // threshold             — min fitted peak height above baseline
      110,      // window_samples        — fit window
      0.5f,     // min_parabola_r2       — fit must explain this much variance
      1.0f,     // max_deviation_rad     — caps Rp-drift tilt of the (t,Rp,L) curve
      220.0f    // length_estimate_scale — thou per uH (peak * scale = crack_size)
  };
  crackDetectionInit(&crackConfig);

  Serial.println("Burrow-Bot Platform: LDC1101 pipeline initialized");
}

// ===== SERVICE INBOUND SERIAL (non-blocking) =====
// The PC streams two kinds of frame to us over Serial1, asynchronously:
//   * motor command  [0xAA][L i16 BE][R i16 BE][0x55]   -> drive the motors
//   * CLI command     [0xAB][len u8][len ASCII][0x55]    -> serialCommandsProcessLine
// We drain whatever bytes are available each loop() with a small byte-at-a-time
// state machine and apply the latest motor command immediately. NOTHING here
// blocks — that is the whole point of the free-running design: the Platform no
// longer waits for a request before streaming a reply (see loop()). A bad
// length or END byte just drops the frame and resyncs at the next start byte.
namespace {
enum RxState { RX_IDLE, RX_MOTOR, RX_CMD_LEN, RX_CMD_DATA, RX_CMD_END };
RxState rxState = RX_IDLE;
uint8_t rxBuf[96];     // CLI payload (<=95) + NUL, or 5 motor bytes
int rxGot = 0;
int rxCmdLen = 0;
}  // namespace

void serviceSerial1() {
  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();
    switch (rxState) {
      case RX_IDLE:
        if (b == PKT_START) {
          rxState = RX_MOTOR;
          rxGot = 0;
        } else if (b == CMD_START) {
          rxState = RX_CMD_LEN;
        }
        break;                                   // stray byte: ignore / resync
      case RX_MOTOR:
        rxBuf[rxGot++] = b;
        if (rxGot == 5) {                         // 4 data bytes + END
          if (rxBuf[4] == PKT_END) {
            int16_t left = (int16_t)((rxBuf[0] << 8) | rxBuf[1]);
            int16_t right = (int16_t)((rxBuf[2] << 8) | rxBuf[3]);
            setMotors(left, right);
          }
          rxState = RX_IDLE;
        }
        break;
      case RX_CMD_LEN:
        rxCmdLen = b;
        if (rxCmdLen <= 0 || rxCmdLen > 95) {
          rxState = RX_IDLE;                       // won't fit the CLI buffer
        } else {
          rxGot = 0;
          rxState = RX_CMD_DATA;
        }
        break;
      case RX_CMD_DATA:
        rxBuf[rxGot++] = b;
        if (rxGot == rxCmdLen) rxState = RX_CMD_END;
        break;
      case RX_CMD_END:
        if (b == PKT_END) {
          rxBuf[rxCmdLen] = '\0';
          // Same dispatcher as the USB CLI; frames any reply back as 0xAC.
          serialCommandsProcessLine((char*)rxBuf);
        }
        rxState = RX_IDLE;
        break;
    }
  }
}

// ===== SEND EXTENDED LDC RESPONSE =====
void sendLDCResponse(const ldc1101_measurement_t& m, bool crackDetected,
                     const crack_detection_result_t* crack,
                     const serial_command_state_t& state) {
  uint8_t* p = txBuffer;
  p[0] = PKT_START;

  uint64_t timestamp = micros();
  memcpy(p + 1, &timestamp, 8);

  int16_t motors[2] = {currentLeftPWM, currentRightPWM};
  memcpy(p + 9, motors, 4);

  // Raw (Rp, L) by default; when rotation is on, send the sample rotated into
  // the calibrated frame (translate to center, rotate by the calibrated angle,
  // translate back) so the PC sees the flattened cloud. Off -> unchanged raw.
  float rp = m.Rp_ohms;
  float l = m.L_uH;
  if (state.rotated) {
    float centerRp, centerL;
    getRotationCenter(&centerRp, &centerL);
    float rotRp, rotL;
    rotateSample(rp - centerRp, l - centerL, getRotationAngle(), rotRp, rotL);
    rp = rotRp + centerRp;
    l = rotL + centerL;
  }
  memcpy(p + 13, &rp, 4);
  memcpy(p + 17, &l, 4);

  // Crack flags are only trustworthy once the rotation has been calibrated.
  uint8_t flags = 0;
  if (state.rotated) flags |= 0x02;
  if (state.rotated && crackDetected) flags |= 0x01;
  p[21] = flags;

  float crack_size = (crack && state.rotated && crackDetected)
                         ? crack->fit_peak_height * crackDetectionGetLengthEstimateScale()
                         : 0.0f;
  memcpy(p + 22, &crack_size, 4);

  p[26] = PKT_END;
  Serial1.write(txBuffer, 27);
}

// ===== MAIN LOOP =====
// Free-running, NOT request/response: motor/CLI commands are applied every loop
// by serviceSerial1() (so control stays instant), while the sensor read +
// telemetry stream are paced by the CLI `delay` (reading_delay_ms). This mirrors
// the ECLAIR scanner's free-running stream and removes the per-sample USB
// round-trip that throttled the old "send one motor command, wait for one reply"
// handshake.
void loop() {
  // 1. Apply any motor commands / CLI frames the PC has streamed (non-blocking,
  //    every loop so motor response and CLI dispatch never wait on the pacing).
  serviceSerial1();

  // 2. Service the USB CLI (tuning / calibrate / material memory). Non-blocking.
  serialCommandsPoll();
  serial_command_state_t state = serialCommandsGetState();
  serial_command_config_t config = serialCommandsGetConfig();

  uint32_t now = millis();

  // 3. Pace the sensor read + telemetry stream to reading_delay_ms (the CLI
  //    `delay`). millis() wrap is handled by the unsigned subtraction.
  if (now - lastSampleMs >= state.reading_delay_ms) {
    lastSampleMs = now;

    // Read one raw (Rp, L) sample and push it through the detection pipeline.
    ldc1101_measurement_t m = ldc1101_read(config.sensor_c_f);
    appendMeasurement(m.Rp_ohms, m.L_uH);

    crack_detection_result_t crackResult = {};
    bool crackDetected = crackDetectionCheck(&crackResult);

    // Latch the crack LED on (only after calibration); the off-latch below.
    if (state.rotated && crackDetected) {
      ledOn();
      crackLedOffMs = now + kCrackLedHoldMs;
    }

    // Stream the telemetry packet. Drop it rather than block if the link's TX
    // buffer is backed up (the PC drains continuously; a dropped sample is
    // cheaper than stalling the loop).
    if (Serial1.availableForWrite() >= (int)sizeof(txBuffer)) {
      sendLDCResponse(m, crackDetected, &crackResult, state);
    }

    // Optional bench Teleplot on USB (off unless `stream on` was issued).
    if (state.streaming_enabled) {
      telemetryEmitSample(now, &state, crackDetected, &crackResult);
    }
  }

  // 4. Crack LED off-latch runs every loop so it expires on time regardless of
  //    the sample pacing.
  if (crackLedOffMs && now >= crackLedOffMs) {
    ledOff();
    crackLedOffMs = 0;
  }
}

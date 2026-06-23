//  _____  _       _    __
// |  __ \| |     | |  / _|
// | |__) | | __ _| |_| |_ ___  _ __ _ __ ___
// |  ___/| |/ _` | __|  _/ _ \| '__| '_ ` _ \
// | |    | | (_| | |_| || (_) | |  | | | | | |
// |_|    |_|\__,_|\__|_| \___/|_|  |_| |_| |_|
//
// ===== ANTENNA - RESPONDER MODE + ON-DEVICE LDC PIPELINE =====
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
#define PKT_START 0xAA  // motor command frame (PC->antenna) and telemetry frame (antenna->PC)
#define CMD_START 0xAB  // ASCII CLI command frame (PC->antenna): [0xAB][len][ascii][0x55]
#define PKT_END 0x55

// Pins
#define UART_TX_PIN 43
#define UART_RX_PIN 44

// Motor pins (DRV8835 IN/IN mode)
#define AIN1 1
#define AIN2 2
#define BIN1 3
#define BIN2 4

// Antenna status LED (active-high on this board).
#define LED_PIN 21

// ===== LDC PIPELINE BOOT DEFAULTS =====
// All of these are reconfigurable at runtime via the USB CLI; they are just the
// power-on seeds. Values match the burrow-bot's stacked inductor (L = 11.8 uH,
// the smallest of the 11.8 / 42.6 / 90.0 uH stack) and 220 pF tank.
static constexpr float kSensorL_H = 11.8e-6f;  // H
static constexpr float kSensorC_F = 220e-12f;  // F
static constexpr float kSensorQ = 30.0f;       // LC tank quality factor

// External mux/switch driven by the driver before configuring. Unused here.
static constexpr int kSwitchEnable = 0;
static constexpr int kSwitchGpio = -1;

// Sample pacing is driven by the PC request rate (~100 Hz), so the CLI's
// reading_delay_ms is unused on this firmware (kept at 0).
static constexpr uint32_t kDefaultReadingDelayMs = 0;

// Onboard LED polarity (XIAO active-low; this antenna LED on pin 21 is active-high).
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
//   [13..16] float   Rp (ohms)         raw, pre-rotation (PC keeps plotting this)
//   [17..20] float   L  (uH)           raw, pre-rotation
//   [21]     uint8   flags  bit0=crack detected, bit1=rotated/calibrated
//   [22..25] float   crack_size (thou) fit peak height * length-estimate scale
//   [26]     0x55  END
static uint8_t txBuffer[27];

// Non-blocking crack indicator: latch the LED on for a short window instead of
// the ECLAIR firmware's blocking ledFlash(), which would stall the ~100 Hz
// request/response loop and trip PC-side response timeouts.
static uint32_t crackLedOffMs = 0;
static constexpr uint32_t kCrackLedHoldMs = 60;

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

  Serial.println("Burrow-Bot antenna: LDC1101 pipeline initialized");
}

// ===== HANDLE CLI COMMAND FRAME =====
// PC -> antenna command frame: [0xAB][len u8][len ASCII bytes][0x55]. Feeds the
// ASCII command into the same dispatcher as the USB CLI (calibrate, rotated
// on|off, save <name>, ...). Bounded waits keep a torn frame from stalling the
// loop; Serial1.setTimeout() (set in setup) caps the payload read.
void handleCommandFrame() {
  // Length byte.
  int len = -1;
  uint32_t start = millis();
  while ((len = Serial1.read()) < 0) {
    if (millis() - start > 20) return;  // torn frame — give up
  }
  if (len <= 0 || len > 95) return;     // must fit the CLI buffer (96 incl. NUL)

  // Payload.
  char buf[96];
  size_t got = Serial1.readBytes(buf, len);
  if (got != (size_t)len) return;
  buf[len] = '\0';

  // End marker.
  int endMarker = -1;
  start = millis();
  while ((endMarker = Serial1.read()) < 0) {
    if (millis() - start > 20) return;
  }
  if (endMarker != PKT_END) return;     // framing error — drop

  serialCommandsProcessLine(buf);
}

// ===== READ MOTOR COMMAND =====
// Blocks until a framed motor command arrives on Serial1, then drives the
// motors. This blocking wait is the request/response sync with the PC.
// Interleaved CLI command frames (0xAB) are dispatched inline as they arrive.
bool readMotorCommand() {
  while (true) {
    int b = Serial1.read();
    if (b == PKT_START) break;                 // motor frame — read it below
    if (b == CMD_START) { handleCommandFrame(); continue; }
    // No data / junk byte: keep the USB CLI responsive and keep waiting.
    serialCommandsPoll();
  }

  // Read the rest of the command: 4 data + 1 end.
  if (Serial1.available() >= 5) {
    uint8_t data[4];
    Serial1.readBytes(data, 4);
    uint8_t endMarker = Serial1.read();

    if (endMarker == PKT_END) {
      int16_t left = (data[0] << 8) | data[1];
      int16_t right = (data[2] << 8) | data[3];
      setMotors(left, right);
      return true;
    }
  }
  return false;
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

  float rp = m.Rp_ohms;
  float l = m.L_uH;
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
void loop() {
  // 1. Wait for the PC's motor command and drive the motors.
  readMotorCommand();

  // 2. Service the USB CLI (tuning / calibrate / material memory). Non-blocking.
  serialCommandsPoll();
  serial_command_state_t state = serialCommandsGetState();
  serial_command_config_t config = serialCommandsGetConfig();

  // 3. Read one raw (Rp, L) sample and push it through the detection pipeline.
  ldc1101_measurement_t m = ldc1101_read(config.sensor_c_f);
  appendMeasurement(m.Rp_ohms, m.L_uH);

  crack_detection_result_t crackResult = {};
  bool crackDetected = crackDetectionCheck(&crackResult);

  // 4. Non-blocking crack LED (only after calibration).
  uint32_t now = millis();
  if (state.rotated && crackDetected) {
    ledOn();
    crackLedOffMs = now + kCrackLedHoldMs;
  } else if (crackLedOffMs && now >= crackLedOffMs) {
    ledOff();
    crackLedOffMs = 0;
  }

  // 5. Reply to the PC with the extended telemetry packet.
  sendLDCResponse(m, crackDetected, &crackResult, state);

  // 6. Optional bench Teleplot on USB (off unless `stream on` was issued).
  if (state.streaming_enabled) {
    telemetryEmitSample(now, &state, crackDetected, &crackResult);
  }
}

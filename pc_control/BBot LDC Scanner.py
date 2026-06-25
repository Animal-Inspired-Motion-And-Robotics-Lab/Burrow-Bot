# ---------------------------------------------------------------------------
# BBot LDC Scanner — drive the Eddy Burrow Bot and view its LDC1101 telemetry
#
# Combines two earlier programs into one cockpit:
#   * "LDC SCANNER V5"            — the ECLAIR scanner GUI (3D R_p/L/t surface,
#                                   phase-space / time trace, crack plot, console)
#   * "AAA Opt BBot Control Code" — the request/response robot driver (Xbox
#                                   joystick -> motor commands, binary telemetry
#                                   read-back, HDF5 + camera recording)
#
# Unlike the ECLAIR scanner, the burrow-bot is NOT a passive stream: the Platform
# only answers when polled. So this app owns the 100 Hz request/response loop.
# Every tick, it sends a motor command (the joystick's, or 0/0 when idle), then
# reads exactly one 27-byte telemetry packet back over the bridge. Those samples
# feed the same live views the ECLAIR scanner used.
#
# Layout (mirrors LDC SCANNER V5):
#   upper-left   3D R_p / L / time surface
#   upper-right  phase-space (R_p vs L)  <->  time trace  (click the x-axis)
#   lower-left   live readout + Platform CLI console + CSV write controls
#   lower-right  crack-detection plot + serial-link connection controls
#
# Two serial channels, exactly as the firmware expects (see CLAUDE.md):
#   * binary motor/telemetry frames travel over the one bridge link this app
#     opens (PC <-USB-> bridge ESP32 <-UART-> platform ESP32);
#   * Platform CLI commands are wrapped as [0xAB][len][ascii][0x55] frames and
#     sent over that SAME link; the Platform frames each reply line back as
#     [0xAC][len][ascii][0x55], which this app demuxes from the telemetry stream
#     and shows in the console (a tick or two later -- not synchronous).
#
# Sections, top to bottom:
#   1. Configuration        2. Pure helpers (parsing-free: geometry only)
#   3. CsvLogger            4. SerialComm + LinkManager (binary link)
#   5. JoystickControl      6. Camera (disabled by default)
#   7. ScannerState         8. Runtime objects
#   9. Qt user interface    10. Control loop + redraw + handlers
# ---------------------------------------------------------------------------

import csv
import math
import os
import queue
import struct
import threading
import time
from collections import deque
from typing import Any

import numpy as np
import serial
from serial.tools import list_ports
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui

# OpenCV backs the optional camera preview. Experiment data is recorded to CSV
# via the GUI's "Write to File" toggle (HDF5 recording was removed). The GUI and
# robot link work without a camera attached.
try:
    import cv2
except Exception as exc:                                    # pragma: no cover
    cv2 = None
    print(f"OpenCV unavailable -- camera/video disabled: {exc}")

# pygame backs the Xbox controller. Optional: without it you can still watch
# telemetry and send CLI commands (the loop just sends a 0/0 motor command).
try:
    import pygame
except Exception as exc:                                    # pragma: no cover
    pygame = None
    print(f"pygame unavailable -- joystick driving disabled: {exc}")

pg.setConfigOptions(antialias=True)


# ---------------------------------------------------------------------------
# 1. Configuration
# ---------------------------------------------------------------------------

# --- Serial link (PC <-> bridge ESP32) -------------------------------------
SERIAL_PORT = "COM5"                # preselected port if detected at launch
BAUDRATE = 460800                   # bridge link baud (matches BBot control code)
BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

# --- Request/response cadence ----------------------------------------------
HERTZ = 100
LOOP_INTERVAL = 1.0 / HERTZ
# The Platform free-runs (streams telemetry continuously), so a tick with no new
# frames is normal -- only warn if the link goes silent this long while connected.
STALE_TELEMETRY_SEC = 0.5

# --- Packet framing (firmware src/platform/main.cpp) -----------------------
PKT_START = 0xAA                    # motor command frame + telemetry frame
CMD_START = 0xAB                    # ASCII CLI frame: [0xAB][len][ascii][0x55]
REPLY_START = 0xAC                  # Platform CLI reply frame: [0xAC][len][ascii][0x55]
PKT_END = 0x55
# Telemetry: [START][ts u64][motorL i16][motorR i16][Rp f][L f][flags][crack_size f][END]
RESP_LEN = 27
FLAG_CRACK = 0x01                   # crack detected this sample (post-calibration)
FLAG_ROTATED = 0x02                 # device rotation/calibration active

# --- CSV output (quick sample log, ECLAIR-style) ---------------------------
CSV_FILE = "bbot_scan.csv"

# --- HDF5 + video experiment recordings (A button) -------------------------
RECORDINGS_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Recordings")
os.makedirs(RECORDINGS_FOLDER, exist_ok=True)
CAMERA_INDEX = 1                    # webcam to record alongside telemetry
# Camera disabled: cv2 cap.read() blocks the 100 Hz control loop until the next
# webcam frame (~33 ms at 30 fps) and buffers stale frames, throttling motor
# commands + telemetry to camera rate. Set True to re-enable preview/recording
# (and consider moving capture to a background thread before doing so).
CAMERA_ENABLED = False

# --- Live data buffers -----------------------------------------------------
MAX_POINTS = 5000                   # ring-buffer length for every sample deque
# Cap the 3D surface mesh size: the ribbon is rebuilt every redraw, so bounding
# the vertex count keeps that cost flat regardless of how many samples have
# accumulated (a full 5000-point rebuild on the GUI thread starved the control
# loop and made driving bursty). Decimated by stride; the newest point is kept.
MAX_SURFACE_POINTS = 1200

# --- Plot / readout tuning -------------------------------------------------
DISPLAY_LAG_POINTS = 1              # skip newest N points to reduce right-edge jitter
RECENT_FADE_POINTS = 100            # length of the highlighted red->white tail
AVERAGE_UPDATE_INTERVAL_SEC = 5.0
RP_ZERO_EPSILON = 1e-12             # R_p span at/below this counts as "flat/zero"
RP_ZERO_FALLBACK_WINDOW = 100       # samples inspected when deciding R_p is flat
INCOMING_HISTORY_LINES = 3          # rolling raw-packet readout depth

# --- Plot line-width scaling (percent; 100 == original width) --------------
SURFACE_TRACE_LINE_WIDTH_PERCENT = 100
RIGHT_PLOT_MAIN_LINE_WIDTH_PERCENT = 200
RIGHT_PLOT_RECENT_LINE_WIDTH_PERCENT = 200
CRACK_PLOT_LINE_WIDTH_PERCENT = 300


def scale_line_width(base_width, percent):
    """Return ``base_width`` scaled by a percentage (100 => unchanged)."""
    try:
        return float(base_width) * (float(percent) / 100.0)
    except (TypeError, ValueError):
        return float(base_width)


# ---------------------------------------------------------------------------
# 2. Pure helpers — surface geometry + a click-to-toggle axis
# ---------------------------------------------------------------------------

def has_usable_rp(rp_vals, eps=RP_ZERO_EPSILON):
    """True when recent R_p values vary by more than ``eps`` (i.e. not flat).

    When the Platform is in RP+L mode but R_p is unused/flat, the phase-space
    plot would collapse to a vertical line; the redraw uses this to fall back to
    a time trace so live L motion is still visible.
    """
    vals = np.asarray(rp_vals, dtype=float)
    if vals.size < 2:
        return False
    window = min(int(RP_ZERO_FALLBACK_WINDOW), vals.size)
    recent = vals[-window:]
    recent = recent[np.isfinite(recent)]
    if recent.size < 2:
        return False
    return float(np.ptp(recent)) > float(eps)


def build_surface_data(x_vals, rp_vals, l_vals):
    """Build the 3D ribbon "curtain" mesh between the live trace and a floor.

    Returns ``(vertices, faces, face_colors, line_pos)`` or ``None`` when there
    are too few points. Each axis is robustly normalized so it stays active even
    with outliers present.
    """
    if len(x_vals) < 2:
        return None

    x_recent = np.asarray(x_vals, dtype=float)
    y_recent = np.asarray(rp_vals, dtype=float)
    z_recent = np.asarray(l_vals, dtype=float)
    if len(x_recent) < 2:
        return None

    # Decimate to a bounded point count so the per-frame mesh rebuild stays cheap
    # (and never grows with the sample buffer). Stride preserves the ribbon shape;
    # the newest sample is always kept so the trace head stays live.
    n_in = len(x_recent)
    if n_in > MAX_SURFACE_POINTS:
        stride = int(np.ceil(n_in / MAX_SURFACE_POINTS))
        idx = np.arange(0, n_in, stride)
        if idx[-1] != n_in - 1:
            idx = np.append(idx, n_in - 1)
        x_recent = x_recent[idx]
        y_recent = y_recent[idx]
        z_recent = z_recent[idx]

    def normalize_centered(vals):
        # Robust scaling keeps each axis active even when outliers are present.
        p_low = float(np.percentile(vals, 5.0))
        p_high = float(np.percentile(vals, 95.0))
        center = 0.5 * (p_low + p_high)
        robust_span = p_high - p_low
        std_span = float(np.std(vals)) * 6.0
        span = max(robust_span, std_span, 1e-9)
        norm = (vals - center) / span
        return np.clip(norm, -0.5, 0.5), center, span

    x_norm, _, _ = normalize_centered(x_recent)
    y_norm, _, _ = normalize_centered(y_recent)
    z_norm, _, _ = normalize_centered(z_recent)

    z_floor = -0.5
    n = len(x_recent)

    vertices = np.empty((2 * n, 3), dtype=np.float32)
    vertices[0::2, 0] = x_norm
    vertices[0::2, 1] = y_norm
    vertices[0::2, 2] = z_floor
    vertices[1::2, 0] = x_norm
    vertices[1::2, 1] = y_norm
    vertices[1::2, 2] = z_norm

    # Vectorized faces: two triangles per quad spanning consecutive samples.
    base = (2 * np.arange(n - 1, dtype=np.uint32))
    faces = np.empty((2 * (n - 1), 3), dtype=np.uint32)
    faces[0::2, 0] = base
    faces[0::2, 1] = base + 1
    faces[0::2, 2] = base + 2
    faces[1::2, 0] = base + 1
    faces[1::2, 1] = base + 3
    faces[1::2, 2] = base + 2

    # Vectorized face colors: per-quad color from the mean z of its two samples,
    # repeated across the quad's two triangles.
    z_color = z_norm + 0.5
    c = 0.5 * (z_color[:-1] + z_color[1:])
    quad_colors = np.empty((n - 1, 4), dtype=np.float32)
    quad_colors[:, 0] = 0.1 + 0.9 * c
    quad_colors[:, 1] = 0.5 * (1.0 - c)
    quad_colors[:, 2] = 1.0 - 0.8 * c
    quad_colors[:, 3] = 0.32
    face_colors = np.repeat(quad_colors, 2, axis=0)

    line_pos = np.column_stack((x_norm, y_norm, z_norm)).astype(np.float32)
    return vertices, faces, face_colors, line_pos


class ToggleAxisItem(pg.AxisItem):
    """An axis that emits ``toggled`` when left-clicked (mode switch)."""
    toggled = QtCore.Signal()

    def mouseClickEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.toggled.emit()
            event.accept()
            return
        super().mouseClickEvent(event)


# ---------------------------------------------------------------------------
# 3. CsvLogger — quick append-only sample log (the F-key / write toggle)
# ---------------------------------------------------------------------------

class CsvLogger:
    """Append-only CSV writer for telemetry samples.

    Separate from the HDF5 experiment recorder: this is the lightweight ECLAIR
    log that also captures the console command/response annotations.
    """

    HEADER = ["timestamp_computer", "timestamp_us", "rp_ohms", "inductance_uH",
              "crack_detected", "crack_size_thou", "calibrated",
              "serial_out", "response"]

    def __init__(self, filename):
        self._file = None
        self._writer = None
        self.path = ""
        self.set_output_file(filename)

    @staticmethod
    def _normalize(filename):
        name = str(filename).strip()
        if not name:
            name = "bbot_scan.csv"
        if not name.lower().endswith(".csv"):
            name += ".csv"
        return name

    def set_output_file(self, filename):
        """Switch to ``filename`` (normalized), writing a header if it is new."""
        self.path = self._normalize(filename)
        if self._file is not None and not self._file.closed:
            self._file.close()
        self._file = open(self.path, "a", newline="")
        self._writer = csv.writer(self._file)
        if os.path.getsize(self.path) == 0:
            self._writer.writerow(self.HEADER)

    def write_sample(self, ts_us, rp, l, crack_detected, crack_size, calibrated,
                     serial_out="", response=""):
        """Append one sample row, stamping the host clock, and flush."""
        self._writer.writerow([
            f"{time.time():.3f}", ts_us, rp, l,
            int(bool(crack_detected)), crack_size, int(bool(calibrated)),
            serial_out, response,
        ])
        self._file.flush()

    @property
    def basename(self):
        return os.path.basename(self.path)

    def close(self):
        if self._file is not None and not self._file.closed:
            self._file.close()


# ---------------------------------------------------------------------------
# 4. SerialComm (binary link) + LinkManager (connection lifecycle)
# ---------------------------------------------------------------------------

class SerialComm:
    """The binary motor/telemetry link to the Platform via the bridge ESP32.

    Ported from the BBot control code. Owns one pyserial port and the
    request/response framing: motor commands out, 27-byte telemetry frames back,
    plus the [0xAB] ASCII CLI frame.
    """

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=0.001)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.read_buffer = b""
        self.responses = []                 # decoded Platform CLI reply lines, drained by the UI
        time.sleep(1)                       # let the bridge/Platform settle
        print(f"Serial connected at {baudrate} baud")

    def send_motor_command(self, left, right):
        """Send framed motor command: [START][L i16 BE][R i16 BE][END]."""
        left = max(min(int(left), 32767), -32768)
        right = max(min(int(right), 32767), -32768)
        packet = bytearray(6)
        packet[0] = PKT_START
        packet[1:3] = struct.pack(">h", left)
        packet[3:5] = struct.pack(">h", right)
        packet[5] = PKT_END
        self.ser.write(packet)

    def send_command(self, text):
        """Send an ASCII CLI command frame: [0xAB][len][ascii][0x55].

        Goes into the same dispatcher as the Platform's USB CLI, so 'calibrate',
        'rotated on', 'q 15', 'save al', ... all work. No telemetry reply.
        """
        payload = text.encode("ascii", "ignore")[:95]
        packet = bytearray([CMD_START, len(payload)])
        packet += payload
        packet.append(PKT_END)
        self.ser.write(packet)

    def drain_packets(self):
        """Non-blocking: return every complete telemetry frame available now.

        The Platform free-runs (streams telemetry continuously) rather than
        answering one reply per request, so each call reads whatever bytes have
        arrived and returns a list of ``(ts, motors, rp, l, flags, crack_size)``
        tuples, oldest first. CLI reply frames (0xAC) interleaved on the same
        link are decoded onto ``self.responses``. NaN Rp/L is kept (not dropped)
        so a dead/misconfigured sensor stays visible upstream. Returns [] when
        nothing complete is buffered -- never blocks.
        """
        if self.ser.in_waiting:
            self.read_buffer += self.ser.read(self.ser.in_waiting)
        packets = []
        while True:
            packet = self._consume_frames()
            if packet is None:
                break
            packets.append(packet)
        return packets

    def _consume_frames(self):
        """Parse complete frames from the front of read_buffer, in order.

        Reply frames are decoded onto ``self.responses``; the first telemetry
        frame found is returned (``(ts, motors, rp, l, flags, crack_size)``) and
        parsing stops there. Unrecognized bytes are dropped to resync. Returns
        None when no telemetry frame is ready (buffer empty or only partial).
        """
        buf = self.read_buffer
        n = len(buf)
        i = 0
        result = None
        while i < n:
            b = buf[i]
            if b == PKT_START:
                if n - i < RESP_LEN:
                    break                                   # incomplete; wait
                if buf[i + RESP_LEN - 1] == PKT_END:
                    data = buf[i + 1:i + RESP_LEN - 1]
                    try:
                        ts, = struct.unpack("<Q", data[0:8])
                        motors = struct.unpack("<hh", data[8:12])
                        rp, = struct.unpack("<f", data[12:16])
                        l, = struct.unpack("<f", data[16:20])
                        flags = data[20]
                        crack_size, = struct.unpack("<f", data[21:25])
                        result = (ts, motors, rp, l, flags, crack_size)
                        i += RESP_LEN
                        break
                    except struct.error:
                        i += 1                               # malformed; resync
                        continue
                i += 1                                       # bad END; resync
                continue
            if b == REPLY_START:
                if n - i < 2:
                    break                                    # need length byte
                length = buf[i + 1]
                frame_end = i + 2 + length                   # index of END byte
                if n <= frame_end:
                    break                                    # payload incomplete
                if buf[frame_end] == PKT_END:
                    text = buf[i + 2:frame_end].decode("ascii", "replace")
                    self.responses.append(text)
                    i = frame_end + 1
                    continue
                i += 1                                       # bad frame; resync
                continue
            i += 1                                           # junk byte; drop
        self.read_buffer = buf[i:]
        return result

    def drain_responses(self):
        """Return and clear queued Platform CLI reply lines."""
        if not self.responses:
            return []
        lines = self.responses
        self.responses = []
        return lines

    def clear_buffer(self):
        self.ser.reset_input_buffer()
        self.read_buffer = b""
        self.responses = []

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class LinkManager:
    """Owns the (re)connectable binary link for the connection panel.

    Mirrors the role the ECLAIR scanner's SerialManager played for its UI --
    is_connected / status_text / on_change / connect / disconnect -- but wraps
    the binary SerialComm instead of a line-based port.
    """

    def __init__(self, baud_default):
        self.comm = None                    # SerialComm or None
        self.port_name = None
        self.baudrate = baud_default
        self.status_text = "Disconnected"
        self.on_change = None               # optional callback() set by the UI

    @property
    def is_connected(self):
        return self.comm is not None

    @staticmethod
    def available_ports():
        """Return ``(device, description)`` for all detected serial ports."""
        return [(p.device, p.description) for p in list_ports.comports()]

    def connect(self, port_name, baudrate):
        """Open ``port_name`` at ``baudrate``. Returns True on success."""
        self.disconnect()
        try:
            self.comm = SerialComm(port_name, baudrate)
        except (serial.SerialException, ValueError, OSError) as exc:
            self.comm = None
            self.status_text = f"Connect failed: {exc}"
            self._notify()
            return False
        self.port_name = port_name
        self.baudrate = baudrate
        self.status_text = f"Connected: {port_name} @ {baudrate}"
        self._notify()
        return True

    def disconnect(self, reason=None):
        if self.comm is not None:
            self.comm.close()
        self.comm = None
        self.status_text = f"Disconnected ({reason})" if reason else "Disconnected"
        self._notify()

    def close(self):
        self.disconnect()

    def _notify(self):
        if self.on_change is not None:
            self.on_change()


# ---------------------------------------------------------------------------
# 5. JoystickControl — Xbox driving + button bindings
# ---------------------------------------------------------------------------

class _ControllerReader(threading.Thread):
    """Background pygame poller -- owns ALL pygame access.

    pygame's event pump blocks ~100+ ms per call on this setup (Windows XInput),
    so it must never run on the GUI thread (it froze typing, redraw, and the
    control loop). This thread keeps the latest stick axes and queues button-down
    events; the GUI thread reads them without touching pygame.
    """

    def __init__(self):
        super().__init__(daemon=True)
        self._lock = threading.Lock()
        self._axis1 = 0.0
        self._axis3 = 0.0
        self.buttons = queue.Queue()
        self.ready = threading.Event()
        self.failed = False
        self.name = ""
        self._running = True

    def run(self):
        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                print("No joystick found")
                self.failed = True
                self.ready.set()
                return
            js = pygame.joystick.Joystick(0)
            js.init()
            self.name = js.get_name()
            print(f"Controller: {self.name}")
        except Exception as exc:
            print(f"Joystick init failed: {exc}")
            self.failed = True
            self.ready.set()
            return

        self.ready.set()
        while self._running:
            for event in pygame.event.get():        # also pumps; the slow part
                if event.type == pygame.JOYBUTTONDOWN:
                    self.buttons.put(event.button)
            try:
                a1 = js.get_axis(1)
                a3 = js.get_axis(3)
            except Exception:
                a1 = a3 = 0.0
            with self._lock:
                self._axis1 = a1
                self._axis3 = a3
            time.sleep(0.005)

    def axes(self):
        with self._lock:
            return self._axis1, self._axis3

    def stop(self):
        self._running = False


class JoystickControl:
    """Xbox controller -> tank-drive PWM + button binds.

    All pygame I/O runs on a background _ControllerReader thread (its event pump
    is ~100 ms/call here and would freeze the GUI). This object stays on the GUI
    thread: update() turns the latest thread-read axes into PWM instantly, and
    process_buttons() drains queued presses so their handlers run GUI-side.
    Button handlers reach the GUI through module-level functions
    (place_material_marker / reset_view / toggle_csv_recording / request_quit)
    and queue Platform CLI commands on the shared ``command_queue``.
    """

    def __init__(self):
        if pygame is None:
            raise RuntimeError("pygame not available")
        self._reader = _ControllerReader()
        self._reader.start()
        self._reader.ready.wait(timeout=2.0)
        if self._reader.failed or not self._reader.is_alive():
            raise RuntimeError("No joystick found")

        self.power = 50
        self.polarity = 1
        self.rotated = False                # mirrors device rotation (telemetry flag)

    def update(self):
        """Return ``(left_pwm, right_pwm)`` from the latest (thread-read) axes."""
        axis1, axis3 = self._reader.axes()
        threshold = 0.1
        left_y = axis1 * self.polarity
        right_y = axis3 * self.polarity
        if abs(left_y) < threshold:
            left_y = 0
        if abs(right_y) < threshold:
            right_y = 0
        return (int(left_y * 255 * self.power / 100),
                int(right_y * 255 * self.power / 100))

    def process_buttons(self):
        """Drain queued controller presses and run their handlers (GUI thread)."""
        while True:
            try:
                button = self._reader.buttons.get_nowait()
            except queue.Empty:
                break
            self.handle_button(button)

    def handle_button(self, button):
        if button == 4:                     # LB - power down
            self.power = max(0, self.power - 10)
            print(f"\nPower: {self.power}%")
        elif button == 5:                   # RB - power up
            self.power = min(100, self.power + 10)
            print(f"\nPower: {self.power}%")
        elif button == 2:                   # B - place material marker
            place_material_marker()
        elif button == 3:                   # A - toggle CSV experiment recording
            toggle_csv_recording()
        elif button == 0:                   # X - reset all plots
            reset_view()
        elif button == 9:                   # L-stick click - on-device calibrate
            command_queue.append("calibrate")
            print("\n[cmd] calibrate")
        elif button == 10:                  # R-stick click - toggle rotation
            self.rotated = not self.rotated
            command_queue.append("rotated on" if self.rotated else "rotated off")
            print(f"\n[cmd] rotated {'on' if self.rotated else 'off'}")
        elif button == 8:                   # Back/View - quit
            request_quit()

    def stop(self):
        self._reader.stop()


# ---------------------------------------------------------------------------
# 6. Camera — optional webcam preview (disabled by default)
# ---------------------------------------------------------------------------

cap = None
camera_ok = False
camera_fps = 30
camera_width = 320
camera_height = 240
video_out = None
current_video_path = None
frame_counter = 0

if CAMERA_ENABLED and cv2 is not None:
    try:
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
        camera_ok = bool(cap.isOpened())
        if camera_ok:
            fps = int(cap.get(cv2.CAP_PROP_FPS))
            camera_fps = fps if fps > 0 else 30
            camera_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or camera_width
            camera_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or camera_height
            print(f"Camera: {camera_width}x{camera_height} at {camera_fps}fps")
        else:
            print(f"Camera index {CAMERA_INDEX} did not open -- preview/recording disabled")
    except Exception as exc:
        print(f"Camera init failed: {exc}")
        cap = None
        camera_ok = False
else:
    print("Camera disabled (CAMERA_ENABLED=False) -- preview/recording off, "
          "control loop runs at full rate")


def grab_camera_frame(record):
    """Grab + preview one frame; write it to video when ``record``.

    Returns ``(host_time, frame_number)`` when a frame was captured, else None.
    """
    global frame_counter
    if not (camera_ok and cap is not None):
        return None
    ret, frame = cap.read()
    if not ret:
        return None
    frame_counter += 1
    cv2.imshow("Camera", frame)
    cv2.waitKey(1)
    if record and video_out is not None:
        video_out.write(frame)
    return (time.time(), frame_counter)


# ---------------------------------------------------------------------------
# 7. ScannerState — live sample buffers + readout text
# ---------------------------------------------------------------------------

class ScannerState:
    """All mutable runtime state: sample ring buffers, crack events, readouts."""

    def __init__(self):
        # Live sample ring buffers (sensor1 = R_p, sensor2 = L).
        self.timestamps = deque(maxlen=MAX_POINTS)   # seconds (device micros/1e6)
        self.sensor1 = deque(maxlen=MAX_POINTS)      # R_p (ohms)
        self.sensor2 = deque(maxlen=MAX_POINTS)      # L (uH)
        self.crack_times = deque(maxlen=MAX_POINTS)
        self.crack_mags = deque(maxlen=MAX_POINTS)
        self.crack_sizes = deque(maxlen=MAX_POINTS)

        # Control flags.
        self.paused = False
        self.write_to_file_enabled = False

        # Readout text + latest decoded values.
        self.incoming_history = deque(maxlen=INCOMING_HISTORY_LINES)
        self.readout_text = "Incoming: waiting for data..."
        self.average_text = f"Avg last {RECENT_FADE_POINTS}: waiting for data..."
        self.latest_crack_size = None
        self.last_crack_flag = False                 # rising-edge detect
        self.pending_serial_out = ""
        self.pending_response = ""
        self.last_average_update_time = 0.0

        # Diagnostics surfaced in the raw readout.
        self.good_count = 0
        self.nan_count = 0
        self.timeout_count = 0
        self.last_packet_monotonic = time.monotonic()   # for stream-stall detection

        # View tracking.
        self.xy_start_index = 0
        self.right_x_mode = "RP"                     # "RP" or "TIME"
        self.right_plot_auto_time_fallback = False
        self.crack_y_mode = "crack_size"             # "crack_size" or "mag"
        self.ui_start_monotonic = time.monotonic()

    def reset(self):
        """Clear buffered data so all plots restart from a clean state."""
        self.timestamps.clear()
        self.sensor1.clear()
        self.sensor2.clear()
        self.crack_times.clear()
        self.crack_mags.clear()
        self.crack_sizes.clear()
        self.xy_start_index = 0
        self.last_crack_flag = False
        self.ui_start_monotonic = time.monotonic()
        self.average_text = f"Avg last {RECENT_FADE_POINTS}: waiting for data..."
        self.last_average_update_time = time.monotonic()

    def stage_command_exchange(self, serial_out, response):
        """Queue one command/response pair to annotate the next logged sample."""
        self.pending_serial_out = str(serial_out or "")
        self.pending_response = str(response or "")

    def consume_pending_command_exchange(self):
        serial_out = self.pending_serial_out
        response = self.pending_response
        self.pending_serial_out = ""
        self.pending_response = ""
        return serial_out, response

    def register_crack(self, t, crack_size):
        """Record a crack event for the crack plot (one bar at time ``t``)."""
        size = float(crack_size) if crack_size and math.isfinite(crack_size) else 0.0
        self.crack_times.append(float(t))
        # Only crack_size comes from this firmware; use it for both y-modes so
        # the bar has height regardless of which mode the crack axis shows.
        self.crack_mags.append(size if size > 0 else 1.0)
        self.crack_sizes.append(size)
        self.latest_crack_size = size

    def ingest_sample(self, t, rp, l, crack_size, crack_detected, calibrated):
        """Store one finite sample and refresh the decoded readout line."""
        self.timestamps.append(t)
        self.sensor1.append(rp)
        self.sensor2.append(l)
        cs_text = (f"{crack_size:.1f}" if crack_size is not None
                   and math.isfinite(crack_size) else "n/a")
        self.readout_text = (
            f"Incoming: t={t:.3f}s | Rp={rp:.2f} ohm | L={l:.3f} uH | "
            f"crack_size={cs_text} thou | calibrated={int(bool(calibrated))} | "
            f"crack={int(bool(crack_detected))}"
        )


# ---------------------------------------------------------------------------
# 8. Runtime objects
# ---------------------------------------------------------------------------

link = LinkManager(BAUDRATE)
csv_logger = CsvLogger(CSV_FILE)
state = ScannerState()
command_queue = []                  # Platform CLI commands, drained each control tick

# Joystick is optional -- without it the loop still polls telemetry (sending a
# 0/0 motor command), which is exactly what calibration needs (data flowing).
joystick: Any = None
if pygame is not None:
    try:
        joystick = JoystickControl()
    except Exception as exc:
        print(f"Joystick unavailable -- driving disabled: {exc}")
        joystick = None


# ---------------------------------------------------------------------------
# 9. Qt user interface
# ---------------------------------------------------------------------------

# High-DPI rounding (Qt6 enables HiDPI automatically; just set rounding policy).
try:
    QtWidgets.QApplication.setHighDpiScaleFactorRoundingPolicy(
        QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )
except Exception:
    pass

app = QtWidgets.QApplication([])

app.setStyleSheet(
    """
    QComboBox, QDoubleSpinBox {
        background-color: #1e1e1e; color: #e6e6e6;
        border: 1px solid #555555; border-radius: 3px; padding: 2px 6px;
    }
    QComboBox:disabled, QDoubleSpinBox:disabled {
        color: #888888; background-color: #161616;
    }
    QComboBox QAbstractItemView {
        background-color: #1e1e1e; color: #e6e6e6;
        border: 1px solid #555555; outline: none;
        selection-background-color: #3a6ea5; selection-color: #ffffff;
    }
    """
)

main_widget = QtWidgets.QWidget()
main_widget.setWindowTitle("BBot LDC Scanner")
main_layout = QtWidgets.QVBoxLayout(main_widget)
main_layout.setContentsMargins(6, 6, 6, 6)
main_layout.setSpacing(6)

# Top row: 3D surface (left) + phase-space/time plot (right).
top_row_layout = QtWidgets.QHBoxLayout()
top_row_layout.setContentsMargins(0, 0, 0, 0)
top_row_layout.setSpacing(6)
main_layout.addLayout(top_row_layout, 1)

# --- Upper-left: live 3D surface -------------------------------------------
surface_container = QtWidgets.QWidget()
surface_layout = QtWidgets.QVBoxLayout(surface_container)
surface_layout.setContentsMargins(0, 0, 0, 0)
surface_layout.setSpacing(2)

surface_view = gl.GLViewWidget()
surface_view.setMinimumSize(520, 340)
surface_view.opts["distance"] = 2.8
surface_view.opts["elevation"] = 22
surface_view.opts["azimuth"] = -35
surface_layout.addWidget(surface_view, 1)

surface_grid = gl.GLGridItem()
surface_grid.setSize(1.4, 1.4)
surface_grid.setSpacing(0.1, 0.1)
surface_view.addItem(surface_grid)

surface_axis = gl.GLAxisItem()
surface_axis.setSize(1.0, 1.0, 1.0)
surface_view.addItem(surface_axis)

_axis_font = QtGui.QFont("Helvetica", 11, QtGui.QFont.Weight.Bold)
surface_view.addItem(gl.GLTextItem(pos=np.array([0.56, 0.0, 0.0], dtype=float),
                                   text="Time", color=QtGui.QColor(255, 80, 80), font=_axis_font))
surface_view.addItem(gl.GLTextItem(pos=np.array([0.0, 0.56, 0.0], dtype=float),
                                   text="R_p", color=QtGui.QColor(80, 200, 80), font=_axis_font))
surface_view.addItem(gl.GLTextItem(pos=np.array([0.0, 0.0, 0.56], dtype=float),
                                   text="L", color=QtGui.QColor(80, 160, 255), font=_axis_font))

# Bootstrap quad shown until enough samples arrive to build a real ribbon.
_bootstrap_vertices = np.array(
    [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [1.0, 0.0, 1.0]],
    dtype=np.float32,
)
_bootstrap_faces = np.array([[0, 1, 2], [1, 3, 2]], dtype=np.uint32)
_bootstrap_face_colors = np.array(
    [[0.2, 0.6, 1.0, 0.32], [0.2, 0.6, 1.0, 0.32]], dtype=np.float32,
)

surface_meshdata = gl.MeshData(vertexes=_bootstrap_vertices, faces=_bootstrap_faces)
surface_meshdata.setFaceColors(_bootstrap_face_colors)
surface_item = gl.GLMeshItem(meshdata=surface_meshdata, smooth=False, drawEdges=False, drawFaces=True)
surface_item.setGLOptions("translucent")
surface_view.addItem(surface_item)

surface_trace = gl.GLLinePlotItem(
    pos=np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 1.0]], dtype=np.float32),
    color=(1.0, 0.65, 0.2, 1.0),
    width=scale_line_width(2.0, SURFACE_TRACE_LINE_WIDTH_PERCENT),
    antialias=True, mode="line_strip",
)
surface_view.addItem(surface_trace)

surface_head = gl.GLScatterPlotItem(
    pos=np.array([[1.0, 0.0, 1.0]], dtype=np.float32),
    color=(1.0, 1.0, 1.0, 1.0), size=8.0,
)
surface_view.addItem(surface_head)

# Faint controller-binding cheat sheet pinned to the lower-left of the 3D view.
# Parented to surface_view (a QWidget) so it floats over the GL canvas; mouse-
# transparent so it never steals the click/drag used to orbit the camera.
controls_overlay = QtWidgets.QLabel(surface_view)
controls_overlay.setText(
    "L-Stick: left motor   R-Stick: right motor\n"
    "LB / RB: power - / +\n"
    "A: record (HDF5+video)   B: material marker\n"
    "X: reset plots\n"
    "L-click: calibrate   R-click: toggle rotation\n"
    "Back/View: quit"
)
controls_overlay.setStyleSheet(
    "color: rgba(200, 200, 200, 150); font-size: 14px; background: transparent;"
)
controls_overlay.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
controls_overlay.adjustSize()


def _position_controls_overlay():
    """Keep the cheat sheet anchored to the 3D view's lower-left corner."""
    margin = 6
    controls_overlay.adjustSize()
    controls_overlay.move(margin, surface_view.height() - controls_overlay.height() - margin)


# Reposition on resize without clobbering GLViewWidget's own GL viewport handling.
_orig_surface_resize_event = surface_view.resizeEvent


def _surface_resize_event(event):
    _orig_surface_resize_event(event)
    _position_controls_overlay()


surface_view.resizeEvent = _surface_resize_event

top_row_layout.addWidget(surface_container, 1)

# --- Upper-right: phase-space / time trace ---------------------------------
right_container = QtWidgets.QWidget()
right_layout = QtWidgets.QVBoxLayout(right_container)
right_layout.setContentsMargins(0, 0, 0, 0)
right_layout.setSpacing(2)

win = pg.GraphicsLayoutWidget()
right_layout.addWidget(win, 1)
top_row_layout.addWidget(right_container, 1)

win.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
win.setFocus()

bottom_axis = ToggleAxisItem(orientation="bottom")
plot_xy = win.addPlot(title="Phase Space", axisItems={"bottom": bottom_axis})
plot_xy.setLabel("bottom", "R_p (ohm)")
plot_xy.setLabel("left", "L (uH)")
bottom_axis.setToolTip("Click x-axis to toggle between R_p and Time")


def set_right_x_mode(mode):
    """Apply right-plot x-axis mode ("RP"/"TIME") and refresh its title/labels."""
    state.right_x_mode = mode
    if state.right_x_mode == "TIME":
        plot_xy.setTitle("Time Trace")
        plot_xy.setLabel("bottom", "Time (s)")
    elif state.right_plot_auto_time_fallback:
        plot_xy.setTitle("Time Trace (auto fallback)")
        plot_xy.setLabel("bottom", "Time (s, Rp flat/zero)")
    else:
        plot_xy.setTitle("Phase Space")
        plot_xy.setLabel("bottom", "R_p (ohm)")


def toggle_right_x_mode():
    set_right_x_mode("TIME" if state.right_x_mode == "RP" else "RP")


bottom_axis.toggled.connect(toggle_right_x_mode)

initial_xy_view_state = plot_xy.getViewBox().getState(copy=True)
xy_curve = plot_xy.plot(pen=pg.mkPen("r", width=scale_line_width(1.0, RIGHT_PLOT_MAIN_LINE_WIDTH_PERCENT)))
# Pre-build the red->white gradient pens once; the per-segment shade is fixed by
# index, so the redraw only needs setData() (no per-frame mkPen on ~100 curves).
recent_segment_curves = []
_seg_total = max(RECENT_FADE_POINTS - 1, 0)
_seg_width = scale_line_width(3.0, RIGHT_PLOT_RECENT_LINE_WIDTH_PERCENT)
for _i in range(_seg_total):
    _shade = int(255 * _i / max(_seg_total - 1, 1))      # red (oldest) -> white (newest)
    recent_segment_curves.append(
        plot_xy.plot(pen=pg.mkPen((255, _shade, _shade, 255), width=_seg_width))
    )

# Persistent overlay markers on the phase plot (material labels + crack X's).
# Tracked so the per-frame TextItem sweep in update() leaves them alone.
material_marker_items = []
crack_marker_items = []

# Lower row: controls (left) + crack plot & connection (right).
lower_row_layout = QtWidgets.QHBoxLayout()
lower_row_layout.setContentsMargins(0, 0, 0, 0)
lower_row_layout.setSpacing(6)

# --- Lower-right top: crack-event plot -------------------------------------
crack_frame = QtWidgets.QFrame()
crack_frame.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
crack_frame.setStyleSheet("QFrame { border: 1px dashed #666; border-radius: 4px; background: #141414; }")
crack_frame_layout = QtWidgets.QVBoxLayout(crack_frame)
crack_frame_layout.setContentsMargins(8, 8, 8, 8)
crack_frame_layout.setSpacing(0)
crack_win = pg.GraphicsLayoutWidget()
crack_frame_layout.addWidget(crack_win)
crack_frame.setFixedHeight(242)

crack_left_axis = ToggleAxisItem(orientation="left")
crack_plot = crack_win.addPlot(axisItems={"left": crack_left_axis})
crack_plot.setLabel("bottom", "Time (s)")
crack_plot.setLabel("left", "crack_size (thou)")
crack_plot.showGrid(x=True, y=True, alpha=0.25)
crack_plot.setYRange(0.0, 1.0, padding=0.0)
crack_curve = crack_plot.plot(
    [], [], pen=pg.mkPen((255, 190, 140, 230), width=scale_line_width(1.0, CRACK_PLOT_LINE_WIDTH_PERCENT)),
    connect="pairs",
)
crack_left_axis.setToolTip("Click y-axis to toggle between crack_size and mag")


def set_crack_y_mode(mode):
    state.crack_y_mode = mode
    crack_plot.setLabel("left", "crack_size (thou)" if mode == "crack_size" else "mag")


def toggle_crack_y_mode():
    set_crack_y_mode("mag" if state.crack_y_mode == "crack_size" else "crack_size")


crack_left_axis.toggled.connect(toggle_crack_y_mode)

# --- Lower-right bottom: connection controls -------------------------------
connection_frame = QtWidgets.QFrame()
connection_frame.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
connection_frame.setStyleSheet("QFrame { border: 1px dashed #666; border-radius: 4px; background: #141414; }")
connection_layout = QtWidgets.QVBoxLayout(connection_frame)
connection_layout.setContentsMargins(8, 6, 8, 6)
connection_layout.setSpacing(4)

port_combo = QtWidgets.QComboBox()
port_combo.setMinimumWidth(160)
port_combo.setToolTip("Detected serial ports (bridge ESP32)")

refresh_button = QtWidgets.QPushButton("Refresh")
refresh_button.setMinimumWidth(80)

baud_combo = QtWidgets.QComboBox()
for _rate in BAUD_RATES:
    baud_combo.addItem(str(_rate), _rate)
_default_baud_idx = baud_combo.findData(BAUDRATE)
if _default_baud_idx >= 0:
    baud_combo.setCurrentIndex(_default_baud_idx)
baud_combo.setToolTip("Baud rate")

connect_button = QtWidgets.QPushButton("Connect")
connect_button.setMinimumWidth(100)

# Second button: Xbox-controller status / rescan. Click to (re)scan for the
# controller -- useful when it was unplugged or attached after launch.
controller_button = QtWidgets.QPushButton("Controller [disconnected]")
controller_button.setMinimumWidth(100)
controller_button.setToolTip("Click to (re)scan for the Xbox controller")

connection_row1 = QtWidgets.QHBoxLayout()
connection_row1.setContentsMargins(0, 0, 0, 0)
connection_row1.setSpacing(4)
connection_row1.addWidget(port_combo, 1)
connection_row1.addWidget(refresh_button)

connection_row2 = QtWidgets.QHBoxLayout()
connection_row2.setContentsMargins(0, 0, 0, 0)
connection_row2.setSpacing(4)
connection_row2.addWidget(baud_combo)
connection_row2.addWidget(connect_button, 1)
connection_row2.addWidget(controller_button, 1)

connection_status_label = QtWidgets.QLabel(link.status_text)
connection_status_label.setStyleSheet("font-size: 10px; color: #bbbbbb;")

connection_layout.addLayout(connection_row1)
connection_layout.addLayout(connection_row2)
connection_layout.addWidget(connection_status_label)


def refresh_ports():
    """Re-scan the system for serial ports and repopulate the dropdown."""
    port_combo.clear()
    for device, desc in LinkManager.available_ports():
        port_combo.addItem(device, device)
        port_combo.setItemData(port_combo.count() - 1, desc, QtCore.Qt.ItemDataRole.ToolTipRole)
    default_idx = port_combo.findData(SERIAL_PORT)
    if default_idx >= 0:
        port_combo.setCurrentIndex(default_idx)


def toggle_connection():
    """Connect or disconnect the binary link to the bridge."""
    if link.is_connected:
        link.disconnect()
        return
    port_name = port_combo.currentData() or port_combo.currentText()
    if not port_name:
        link.status_text = "No port selected"
        set_connection_ui_state()
        return
    baud_value = baud_combo.currentData()
    if baud_value is None:
        baud_value = int(baud_combo.currentText())
    if not link.connect(port_name, int(baud_value)):
        refresh_ports()


def set_connection_ui_state():
    """Sync the connection cluster with the link's state."""
    connected = link.is_connected
    connect_button.setText("Disconnect" if connected else "Connect")
    connection_status_label.setText(link.status_text)
    port_combo.setEnabled(not connected)
    baud_combo.setEnabled(not connected)
    refresh_button.setEnabled(not connected)


def update_controller_button():
    """Reflect whether an Xbox controller is currently connected."""
    connected = joystick is not None
    controller_button.setText(
        f"Controller [{'connected' if connected else 'disconnected'}]"
    )


def refresh_controller():
    """(Re)scan for the Xbox controller: connect one that appeared, drop one that left."""
    global joystick
    if pygame is None:
        update_controller_button()
        return
    if joystick is None:
        # Force pygame to re-enumerate so a controller plugged in after launch
        # (no event pump runs while disconnected) is actually picked up.
        try:
            pygame.joystick.quit()
            pygame.joystick.init()
        except Exception:
            pass
        try:
            if pygame.joystick.get_count() > 0:
                joystick = JoystickControl()
        except Exception as exc:
            print(f"Controller connect failed: {exc}")
            joystick = None
    else:
        # Already connected; verify it is still present (control loop pumps events).
        try:
            if pygame.joystick.get_count() <= 0:
                joystick = None
                print("Controller disconnected")
        except Exception:
            joystick = None
    update_controller_button()


refresh_button.clicked.connect(refresh_ports)
connect_button.clicked.connect(toggle_connection)
controller_button.clicked.connect(refresh_controller)
link.on_change = set_connection_ui_state
refresh_ports()
set_connection_ui_state()
update_controller_button()

# Stack the crack plot above the connection panel in the lower-right cell.
right_lower_container = QtWidgets.QWidget()
right_lower_layout = QtWidgets.QVBoxLayout(right_lower_container)
right_lower_layout.setContentsMargins(0, 0, 0, 0)
right_lower_layout.setSpacing(6)
right_lower_layout.addWidget(crack_frame)
right_lower_layout.addWidget(connection_frame)
lower_row_layout.addWidget(right_lower_container, 1)

# --- Lower-left: readouts + Platform CLI console + CSV write controls -------
controls_container = QtWidgets.QWidget()
controls_layout = QtWidgets.QVBoxLayout(controls_container)
controls_layout.setContentsMargins(0, 4, 0, 0)
controls_layout.setSpacing(6)

# Raw per-packet readout (shows NaN/timeout packets too, so a dead LDC is
# visible at a glance).
incoming_line_box = QtWidgets.QPlainTextEdit()
incoming_line_box.setReadOnly(True)
incoming_line_box.setMinimumWidth(320)
incoming_line_box.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
incoming_line_box.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
incoming_line_box.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
_line_h = incoming_line_box.fontMetrics().lineSpacing()
_frame_h = incoming_line_box.frameWidth() * 2
_doc_h = int(incoming_line_box.document().documentMargin() * 2)
incoming_line_box.setFixedHeight(int(_line_h * INCOMING_HISTORY_LINES + _frame_h + _doc_h))
incoming_line_box.setPlainText("Raw packet: waiting for data...")


def append_incoming_line(line):
    """Push a raw packet-summary line into the rolling readout box."""
    if not line:
        return
    state.incoming_history.append(line)
    incoming_line_box.setPlainText("\n".join(state.incoming_history))


readout_label = QtWidgets.QLabel(state.readout_text)
readout_label.setMinimumWidth(320)
average_label = QtWidgets.QLabel(state.average_text)
average_label.setMinimumWidth(320)

readout_layout = QtWidgets.QVBoxLayout()
readout_layout.setContentsMargins(0, 0, 0, 0)
readout_layout.setSpacing(2)
readout_layout.addWidget(incoming_line_box)
readout_layout.addWidget(readout_label)
readout_layout.addWidget(average_label)

readout_container = QtWidgets.QWidget()
readout_container.setLayout(readout_layout)
controls_layout.addWidget(readout_container)

# Platform CLI console (configure the LDC1101: calibrate, q 15, save al, ...).
serial_command_input = QtWidgets.QLineEdit()
serial_command_input.setPlaceholderText("Platform CLI: status  help  q 15  calibrate  rotated on  save al")
serial_command_input.setMinimumWidth(220)

serial_send_button = QtWidgets.QPushButton("Send")
serial_send_button.setMinimumWidth(80)

serial_command_row = QtWidgets.QHBoxLayout()
serial_command_row.setContentsMargins(0, 0, 0, 0)
serial_command_row.setSpacing(4)
serial_command_row.addWidget(serial_command_input)
serial_command_row.addWidget(serial_send_button)

serial_response_box = QtWidgets.QPlainTextEdit()
serial_response_box.setReadOnly(True)
serial_response_box.setMinimumWidth(320)
serial_response_box.setMaximumHeight(120)
serial_response_box.setPlainText(
    "Platform CLI: type a command and press Send.\n"
    "Replies stream back here (may lag a tick or two)."
)

# Rolling transcript for the Platform CLI console (sent commands + replies).
SERIAL_RESPONSE_MAX_LINES = 200
serial_response_history = []


def append_serial_response(text, prefix=""):
    """Append a line to the Platform CLI transcript and scroll to the bottom."""
    serial_response_history.append(f"{prefix}{text}")
    if len(serial_response_history) > SERIAL_RESPONSE_MAX_LINES:
        del serial_response_history[:-SERIAL_RESPONSE_MAX_LINES]
    serial_response_box.setPlainText("\n".join(serial_response_history))
    scrollbar = serial_response_box.verticalScrollBar()
    scrollbar.setValue(scrollbar.maximum())

serial_controls_layout = QtWidgets.QVBoxLayout()
serial_controls_layout.setContentsMargins(0, 0, 0, 0)
serial_controls_layout.setSpacing(2)
serial_controls_layout.addLayout(serial_command_row)
serial_controls_layout.addWidget(serial_response_box)

serial_controls_container = QtWidgets.QWidget()
serial_controls_container.setLayout(serial_controls_layout)
controls_layout.addWidget(serial_controls_container)

# CSV write toggle (defaults OFF) + output filename.
write_toggle_button = QtWidgets.QPushButton("Write to File: OFF")
write_toggle_button.setCheckable(True)
write_toggle_button.setChecked(False)
write_toggle_button.setMinimumWidth(140)


def write_toggle_changed(checked):
    state.write_to_file_enabled = checked
    write_toggle_button.setText("Write to File: ON" if checked else "Write to File: OFF")


write_toggle_button.toggled.connect(write_toggle_changed)

write_file_input = QtWidgets.QLineEdit(csv_logger.path)
write_file_input.setPlaceholderText("CSV filename")
write_file_input.setMinimumWidth(160)
write_file_input.setToolTip("Output CSV file name (press Enter to apply)")

write_file_label = QtWidgets.QLabel(csv_logger.basename)
write_file_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
write_file_label.setStyleSheet("font-size: 10px; color: #bbbbbb;")
write_file_label.setToolTip(csv_logger.path)


def apply_csv_filename():
    csv_logger.set_output_file(write_file_input.text())
    write_file_input.setText(csv_logger.path)
    write_file_label.setText(csv_logger.basename)
    write_file_label.setToolTip(csv_logger.path)


write_file_input.editingFinished.connect(apply_csv_filename)

write_controls_layout = QtWidgets.QVBoxLayout()
write_controls_layout.setContentsMargins(0, 0, 0, 0)
write_controls_layout.setSpacing(2)
write_controls_layout.addWidget(write_toggle_button)
write_controls_layout.addWidget(write_file_input)
write_controls_layout.addWidget(write_file_label)

write_controls_container = QtWidgets.QWidget()
write_controls_container.setLayout(write_controls_layout)
controls_layout.addWidget(write_controls_container)

lower_row_layout.insertWidget(0, controls_container, 1)
main_layout.addLayout(lower_row_layout, 0)
main_widget.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
main_widget.setFocus()
main_widget.resize(1280, 860)
main_widget.show()
_position_controls_overlay()        # anchor the cheat sheet now the view has a size
controls_overlay.show()
controls_overlay.raise_()


# ---------------------------------------------------------------------------
# 10. Control loop + redraw + handlers
# ---------------------------------------------------------------------------

def place_material_marker():
    """Drop a numbered label on the phase plot at the most recent (Rp, L)."""
    if not state.sensor1 or not state.sensor2:
        print("\nNo data to label on phase plot.")
        return
    label = pg.TextItem(f"Material\n {len(material_marker_items) + 1}",
                        anchor=(0.5, 0.5), color=(255, 255, 255), fill=(0, 0, 0, 0))
    label.setPos(state.sensor1[-1], state.sensor2[-1])
    plot_xy.addItem(label)
    material_marker_items.append(label)
    print(f"\nAdded label at ({state.sensor1[-1]:.2f}, {state.sensor2[-1]:.2f})")


def reset_view():
    """Clear all buffers + overlays and restart every plot from scratch."""
    state.reset()

    for marker in material_marker_items + crack_marker_items:
        plot_xy.removeItem(marker)
    material_marker_items.clear()
    crack_marker_items.clear()

    meshdata = gl.MeshData(vertexes=_bootstrap_vertices, faces=_bootstrap_faces)
    meshdata.setFaceColors(_bootstrap_face_colors)
    surface_item.setMeshData(meshdata=meshdata)
    surface_trace.setData(pos=np.array([[0.0, 0.0, 0.0]], dtype=np.float32))
    surface_head.setData(pos=np.array([[0.0, 0.0, 0.0]], dtype=np.float32))
    surface_view.opts["center"] = QtGui.QVector3D(0.0, 0.0, 0.0)

    xy_curve.clear()
    for seg_curve in recent_segment_curves:
        seg_curve.setData([], [])
    plot_xy.getViewBox().setState(initial_xy_view_state)

    average_label.setText(state.average_text)
    crack_curve.setData([], [])
    print("\nPlots reset")


def request_quit():
    """Stop the robot and close the app cleanly (Back button)."""
    if link.is_connected:
        try:
            link.comm.send_motor_command(0, 0)
        except Exception:
            pass
    app.quit()


def toggle_csv_recording():
    """Flip the GUI's CSV "Write to File" toggle (bound to the controller A button)."""
    write_toggle_button.setChecked(not write_toggle_button.isChecked())


def keyPressEvent(event):
    """Space=reset, P=pause, F=toggle CSV write, 1/2/3=3D views."""
    key = event.key()
    if key == QtCore.Qt.Key.Key_Space:
        reset_view()
    elif key == QtCore.Qt.Key.Key_P:
        state.paused = not state.paused
        print("Paused" if state.paused else "Resumed")
    elif key == QtCore.Qt.Key.Key_F:
        write_toggle_button.setChecked(not write_toggle_button.isChecked())
    elif key == QtCore.Qt.Key.Key_1:                 # top-down (look down Z)
        surface_view.opts["elevation"] = 90
        surface_view.opts["azimuth"] = 0
        surface_view.update()
    elif key == QtCore.Qt.Key.Key_2:                 # front (look along Y)
        surface_view.opts["elevation"] = 0
        surface_view.opts["azimuth"] = 0
        surface_view.update()
    elif key == QtCore.Qt.Key.Key_3:                 # left side (look along X)
        surface_view.opts["elevation"] = 0
        surface_view.opts["azimuth"] = 90
        surface_view.update()


main_widget.keyPressEvent = keyPressEvent
win.keyPressEvent = keyPressEvent
surface_view.keyPressEvent = keyPressEvent


def send_serial_command():
    """Queue the typed CLI command for the control loop to frame and send."""
    command = serial_command_input.text().strip()
    if not command:
        return
    if not link.is_connected:
        append_serial_response("Not connected -- connect the link first.", prefix="! ")
        return
    command_queue.append(command)
    state.stage_command_exchange(command, "")        # annotate next CSV row
    append_serial_response(command, prefix=">> ")
    print(f"\n[CLI] queued -> Platform: {command!r}")
    serial_command_input.clear()
    serial_command_input.setFocus()


serial_send_button.clicked.connect(send_serial_command)
serial_command_input.returnPressed.connect(send_serial_command)


def update_raw_readout(rp, l, flags, crack_size):
    """Refresh the rolling raw-packet readout (incl. NaN packets)."""
    rp_finite = math.isfinite(rp)
    l_finite = math.isfinite(l)
    ok = rp_finite and l_finite
    cal = bool(flags & FLAG_ROTATED)
    crk = bool(flags & FLAG_CRACK)
    tag = "OK " if ok else "NaN"
    rp_str = f"{rp:.2f}" if rp_finite else "NaN"
    l_str = f"{l:.3f}" if l_finite else "NaN"
    cs_str = f"{crack_size:.1f}" if math.isfinite(crack_size) else "NaN"
    append_incoming_line(
        f"[{tag}] Rp={rp_str} ohm  L={l_str} uH  flags=0x{flags:02X} "
        f"(cal={int(cal)}, crack={int(crk)})  crack_size={cs_str}  "
        f"good={state.good_count} nan={state.nan_count} to={state.timeout_count}"
    )


# --- TEMP perf instrumentation (remove once the bottleneck is found) ---------
# Accumulates wall-clock time per labelled section and prints avg/max every 2 s.
# Tells us whether the cost is serial I/O to the bridge (ser_write/ser_drain),
# the joystick read, or the rendering (redraw) -- without guessing.
_PERF = {}
_perf_last_report = [time.monotonic()]


def _perf_add(name, dt):
    s = _PERF.setdefault(name, [0.0, 0, 0.0])
    s[0] += dt
    s[1] += 1
    if dt > s[2]:
        s[2] = dt
    now = time.monotonic()
    if now - _perf_last_report[0] >= 2.0:
        parts = []
        for k in sorted(_PERF):
            tot, cnt, mx = _PERF[k]
            if cnt:
                parts.append(f"{k}: avg={tot / cnt * 1e3:.2f}ms max={mx * 1e3:.2f}ms n={cnt}")
            _PERF[k] = [0.0, 0, 0.0]
        print("[perf] " + " | ".join(parts))
        _perf_last_report[0] = now


def _timed(name, fn, *args):
    t0 = time.perf_counter()
    try:
        return fn(*args)
    finally:
        _perf_add(name, time.perf_counter() - t0)


def control_tick():
    """Push the latest motor command and drain all streamed telemetry.

    Free-running (NOT request/response): the Platform streams telemetry
    continuously, so we send motor/CLI commands fire-and-forget and drain
    whatever frames have arrived without ever blocking on a reply. This is what
    keeps both control and data real-time -- mirrors the ECLAIR scanner's
    free-running stream. Single-threaded with the redraw (separate, slower
    timer), but this tick no longer blocks, so a heavy redraw can't stall it.
    """
    if state.paused or not link.is_connected:
        return

    # Joystick -> motor PWM (0/0 when no controller is attached).
    left = right = 0
    if joystick is not None:
        try:
            left, right = _timed("joystick", joystick.update)
            joystick.process_buttons()      # run queued button binds on this thread
        except Exception as exc:
            print(f"\nJoystick read failed: {exc}")

    # Drain queued Platform CLI commands (console + button binds).
    if command_queue:
        for cmd in list(command_queue):
            try:
                link.comm.send_command(cmd)
            except serial.SerialException:
                link.disconnect("device lost")
                return
        command_queue.clear()

    # Motor command, fire-and-forget (left inverted to match wiring). No reply.
    try:
        _timed("ser_write", link.comm.send_motor_command, left * -1, right)
    except serial.SerialException:
        link.disconnect("device lost")
        return

    # Drain every telemetry frame streamed since the last tick (non-blocking).
    packets = _timed("ser_drain", link.comm.drain_packets)

    # Show any Platform CLI replies that arrived (status/help/OK/ERR...).
    for line in link.comm.drain_responses():
        append_serial_response(line)

    if not packets:
        # No data this tick is normal at the stream cadence; only flag a true
        # stall (link alive but silent for a while).
        if time.monotonic() - state.last_packet_monotonic > STALE_TELEMETRY_SEC:
            state.timeout_count += 1
            state.last_packet_monotonic = time.monotonic()   # rate-limit the warning
            append_incoming_line(
                f"[--] no telemetry for >{STALE_TELEMETRY_SEC:.1f}s  "
                f"good={state.good_count} nan={state.nan_count} to={state.timeout_count}"
            )
        return

    state.last_packet_monotonic = time.monotonic()
    for packet in packets:
        process_packet(packet)

    # Refresh the rolling raw readout once per tick (newest frame) to bound cost.
    _ts, _motors, rp, l, flags, crack_size = packets[-1]
    update_raw_readout(rp, l, flags, crack_size)


def process_packet(packet):
    """Ingest one telemetry frame into the live buffers / crack events / CSV."""
    ts, motors, rp, l, flags, crack_size = packet

    if not (math.isfinite(rp) and math.isfinite(l)):
        state.nan_count += 1                # frame arrived but sensor read failed
        return

    state.good_count += 1
    if not math.isfinite(crack_size):
        crack_size = 0.0
    crack_detected = bool(flags & FLAG_CRACK)
    calibrated = bool(flags & FLAG_ROTATED)
    if joystick is not None:
        joystick.rotated = calibrated       # keep the rotate toggle in sync

    t_sec = ts / 1e6
    state.ingest_sample(t_sec, rp, l, crack_size, crack_detected, calibrated)

    # Rising-edge crack event -> crack plot + persistent 'x' on the phase plot.
    if crack_detected and not state.last_crack_flag:
        state.register_crack(t_sec, crack_size)
        marker = pg.TextItem("x", anchor=(0.5, 0.5), color=(255, 0, 0))
        marker.setPos(rp, l)
        plot_xy.addItem(marker)
        crack_marker_items.append(marker)
        print(f"\n>>> CRACK detected: size~{crack_size:.1f} thou")
    state.last_crack_flag = crack_detected

    # CSV log (quick sample log; annotated with any staged command exchange).
    if state.write_to_file_enabled:
        serial_out, response = state.consume_pending_command_exchange()
        csv_logger.write_sample(ts, rp, l, crack_detected, crack_size, calibrated,
                                serial_out, response)


def update():
    """Redraw timer: recompute readouts and repaint all plots."""
    readout_label.setText(state.readout_text)

    # Re-apply user-configured line widths each frame so style never gets lost.
    xy_curve.setPen(pg.mkPen("r", width=scale_line_width(1.0, RIGHT_PLOT_MAIN_LINE_WIDTH_PERCENT)))
    crack_curve.setPen(pg.mkPen((255, 190, 140, 230), width=scale_line_width(1.0, CRACK_PLOT_LINE_WIDTH_PERCENT)))

    x_all = np.array(state.timestamps)
    y1_all = np.array(state.sensor1)
    y2_all = np.array(state.sensor2)

    now = time.monotonic()
    if now - state.last_average_update_time >= AVERAGE_UPDATE_INTERVAL_SEC:
        avg_count = min(RECENT_FADE_POINTS, len(y1_all))
        if avg_count > 0:
            avg_s1 = float(np.mean(y1_all[-avg_count:]))
            avg_s2 = float(np.mean(y2_all[-avg_count:]))
            state.average_text = f"Avg last {avg_count}: Rp={avg_s1:.4f} | L={avg_s2:.4f}"
        else:
            state.average_text = f"Avg last {RECENT_FADE_POINTS}: waiting for data..."
        state.last_average_update_time = now
    average_label.setText(state.average_text)

    if len(x_all) > 0:
        t_min = float(x_all[0])
        t_now = float(x_all[-1])
    else:
        t_min = 0.0
        t_now = float(time.monotonic() - state.ui_start_monotonic)

    crack_count = min(len(state.crack_times), len(state.crack_mags), len(state.crack_sizes))
    if crack_count > 0:
        crack_times_arr = np.asarray(list(state.crack_times)[-crack_count:], dtype=float)
        crack_mags_arr = np.asarray(list(state.crack_mags)[-crack_count:], dtype=float)
        crack_sizes_arr = np.asarray(list(state.crack_sizes)[-crack_count:], dtype=float)
        crack_vals_arr = crack_sizes_arr if state.crack_y_mode == "crack_size" else crack_mags_arr
        crack_x = np.repeat(crack_times_arr, 2)
        crack_y = np.empty(2 * crack_count, dtype=float)
        crack_y[0::2] = 0.0
        crack_y[1::2] = crack_vals_arr
        crack_curve.setData(crack_x, crack_y)
        crack_plot.setYRange(0.0, max(float(np.max(crack_vals_arr)) * 1.1, 1e-6), padding=0.0)
    else:
        crack_curve.setData([], [])
        crack_plot.setYRange(0.0, 1.0, padding=0.0)

    lag = max(0, int(DISPLAY_LAG_POINTS))
    if lag > 0 and len(x_all) > lag:
        x, y1, y2 = x_all[:-lag], y1_all[:-lag], y2_all[:-lag]
    elif lag == 0:
        x, y1, y2 = x_all, y1_all, y2_all
    else:
        x = y1 = y2 = np.array([], dtype=float)

    if len(x) == 0:
        return

    xy_offset = max(0, state.xy_start_index)
    x_plot = x[xy_offset:]
    y1_plot = y1[xy_offset:]
    y2_plot = y2[xy_offset:]
    if len(x_plot) == 0:
        return

    surface_data = build_surface_data(x_plot, y1_plot, y2_plot)
    if surface_data is not None:
        vertices, faces, face_colors, line_pos = surface_data
        meshdata = gl.MeshData(vertexes=vertices, faces=faces)
        meshdata.setFaceColors(face_colors)
        surface_item.setMeshData(meshdata=meshdata)
        surface_trace.setData(pos=line_pos, width=scale_line_width(2.0, SURFACE_TRACE_LINE_WIDTH_PERCENT))
        surface_head.setData(pos=line_pos[-1:].copy())
        surface_view.opts["center"] = QtGui.QVector3D(0.0, 0.0, 0.0)

    # Sweep transient TextItems off the XY plot, but keep persistent overlays.
    persistent = set(id(m) for m in material_marker_items) | set(id(m) for m in crack_marker_items)
    for item in plot_xy.items[:]:
        if isinstance(item, pg.TextItem) and id(item) not in persistent:
            plot_xy.removeItem(item)

    if state.right_x_mode == "TIME":
        state.right_plot_auto_time_fallback = False
        x_right, y_right = x_plot, y2_plot
    elif has_usable_rp(y1_plot):
        state.right_plot_auto_time_fallback = False
        x_right, y_right = y1_plot, y2_plot
    else:
        # R_p flat/zero: preserve live L tracking by plotting against time.
        state.right_plot_auto_time_fallback = True
        x_right, y_right = x_plot, y2_plot

    set_right_x_mode(state.right_x_mode)

    right_uses_time_x = (state.right_x_mode == "TIME") or state.right_plot_auto_time_fallback
    if right_uses_time_x:
        crack_plot.setXLink(plot_xy)
    else:
        crack_plot.setXLink(None)
        crack_plot.setXRange(t_min, max(t_now, t_min + 1e-6), padding=0.0)

    xy_curve.setData(x_right, y_right)

    # Highlight the most recent trajectory with a red -> white segment gradient.
    tail_count = min(RECENT_FADE_POINTS, len(x_right))
    if tail_count > 1:
        tail_x = x_right[-tail_count:]
        tail_y = y_right[-tail_count:]
        seg_count = min(tail_count - 1, len(recent_segment_curves))
        # Pens are pre-built (fixed gradient); only push new segment endpoints.
        for i in range(seg_count):
            recent_segment_curves[i].setData([tail_x[i], tail_x[i + 1]],
                                             [tail_y[i], tail_y[i + 1]])
        for i in range(seg_count, len(recent_segment_curves)):
            recent_segment_curves[i].setData([], [])
    else:
        for seg_curve in recent_segment_curves:
            seg_curve.setData([], [])


# Fast control loop (motor send + non-blocking telemetry drain) + slower redraw,
# both on the GUI thread, so the redraw is kept cheap to avoid starving control.
control_timer = QtCore.QTimer()
control_timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
control_timer.timeout.connect(lambda: _timed("control_total", control_tick))  # TEMP perf
control_timer.start(int(LOOP_INTERVAL * 1000))      # ~100 Hz

redraw_timer = QtCore.QTimer()
redraw_timer.timeout.connect(lambda: _timed("redraw_total", update))          # TEMP perf
redraw_timer.start(50)                               # ~20 fps


def close_resources():
    if joystick is not None:
        joystick.stop()
    if link.is_connected:
        try:
            link.comm.send_motor_command(0, 0)
        except Exception:
            pass
    link.close()
    csv_logger.close()
    if cap is not None:
        cap.release()
    if video_out is not None:
        video_out.release()
    if cv2 is not None:
        cv2.destroyAllWindows()
    if pygame is not None:
        pygame.quit()
    print("\nShutdown complete")


app.aboutToQuit.connect(close_resources)


def main():
    print("=" * 64)
    print(f"BBot LDC Scanner - free-running stream, motor cmds at {HERTZ} Hz")
    print("=" * 64)
    print("Connect the bridge link (lower-right), then drive / configure:")
    print("  Left Stick: left motor      Right Stick: right motor")
    print("  LB/RB: power -/+            A: toggle CSV write      B: material marker")
    print("  X / Space: reset plots      L-click: calibrate       R-click: toggle rotation")
    print("  Back/View: quit             P: pause   F: CSV write   1/2/3: 3D views")
    print("  Click the phase-plot x-axis to toggle R_p <-> Time; crack-plot y-axis for size <-> mag")
    print("=" * 64)
    app.exec()


if __name__ == "__main__":
    main()

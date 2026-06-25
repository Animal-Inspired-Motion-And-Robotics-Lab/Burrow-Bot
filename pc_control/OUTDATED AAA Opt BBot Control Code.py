####CONTROL CODE - controls the EddyBurrowBot with an XBox controller, outputs the data
#NOTE - You must use an older version of Python for these libraries 
#(Tested on Python 3.12.1)

import pygame
import serial
import struct
import time
import cv2
import h5py
import os
import subprocess
import sys
import math

from datetime import datetime
from typing import Any

# pyqtgraph availability is checked in subprocess to avoid crashes in this process
pyqtgraph_available = False
try:
    result = subprocess.run(
        [sys.executable, "-c", "import pyqtgraph, pyqtgraph.Qt"],
        capture_output=True, text=True, timeout=10
    )
    if result.returncode == 0:
        pyqtgraph_available = True
    else:
        print(f"pyqtgraph unavailable (subprocess): {result.stderr.strip()}")
except Exception as e:
    print(f"pyqtgraph availability subprocess check failed: {e}")

# ===== CONFIGURATION =====
PORT = 'COM5'
BAUDRATE = 460800
HERTZ = 100
LOOP_INTERVAL = 1.0 / HERTZ

# Packet markers
PKT_START = 0xAA   # motor command frame + telemetry frame
CMD_START = 0xAB   # ASCII CLI command frame to the Platform: [0xAB][len][ascii][0x55]
PKT_END = 0x55

# Response packet length (firmware src/platform/main.cpp):
#   [START][ts u64][motorL i16][motorR i16][Rp f][L f][flags u8][crack_size f][END]
#     1      8        2          2          4     4    1         4             1   = 27
RESP_LEN = 27
# flags bit masks
FLAG_CRACK = 0x01   # a crack was detected this sample (post-calibration)
FLAG_ROTATED = 0x02 # device rotation/calibration is active

# Recording folder
#****************************************************************************************************************************
RECORDINGS_FOLDER = r"C:\Users\newsp\Burrow-Bot\Recordings" #THIS NEEDS TO BE CHANGED TO USERS FOLDER
#****************************************************************************************************************************
os.makedirs(RECORDINGS_FOLDER, exist_ok=True)

def get_new_filename(folder, base_name, extension):
    """Generate unique filename in folder"""
    index = 1
    while True:
        new_filename = os.path.join(folder, f"{base_name}_{index}{extension}")
        if not os.path.exists(new_filename):
            return new_filename
        index += 1

# ===== CAMERA SETUP (GLOBAL) =====
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Get camera properties
fps = int(cap.get(cv2.CAP_PROP_FPS))
if fps == 0:
    fps = 30
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(f"Camera: {width}x{height} at {fps}fps")

# Video writer (will be created when recording starts)
video_out = None
current_video_path = None

# ===== GLOBAL PLOTTING STATE =====
# Typed Any: these are seeded to None here and reassigned to live pyqtgraph / Qt
# objects in init_plot_window(). Any keeps the type checker from treating every
# later .setData()/.QApplication/etc. as access on None (the code guards real
# use behind plot_ready).
plot_app: Any = None
plot_layout: Any = None
plot_time: Any = None
plot_scatter: Any = None
plot_curve_time_l: Any = None
plot_curve_time_rp: Any = None
plot_curve_scatter: Any = None
plot_readout: Any = None  # live LDC1101 packet readout (LabelItem)
plot_container: Any = None  # top-level QWidget wrapping plots + CLI input row
plot_cmd_input: Any = None  # QLineEdit for typing Platform CLI commands
plot_cmd_status: Any = None  # QLabel echoing the last command sent
vb_right: Any = None
plot_pg: Any = None
plot_QtGui: Any = None
plot_l = []
plot_rp = []
plot_ready = False

def init_plot_window(joystick=None):
    global plot_app, plot_layout, plot_time, plot_scatter, plot_curve_time_l, plot_curve_time_rp, plot_curve_scatter, plot_readout, vb_right, plot_ready
    global plot_container, plot_cmd_input, plot_cmd_status

    global plot_pg, plot_QtGui
    try:
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtWidgets
        plot_pg = pg
        plot_QtGui = QtWidgets
    except Exception as e:
        print(f"plot window import failed: {e}")
        plot_ready = False
        return

    if plot_app is None:
        try:
            plot_app = plot_QtGui.QApplication([])
        except Exception as e:
            print(f"plot window init failed: {e}")
            plot_ready = False
            return

    try:
        # Create layout with two plots side by side
        plot_layout = plot_pg.GraphicsLayoutWidget()
        plot_layout.resize(1200, 600)
        
        # Left plot: time series
        plot_time = plot_layout.addPlot(row=0, col=0, title="Time Series")
        plot_time.setLabel('bottom', 'Time')
        plot_time.setLabel('left', 'Inductance (uH)')
        plot_time.showAxis('right')
        plot_time.getAxis('right').setLabel('Rp (Ohms)')
        
        # Create viewbox for right axis (Rp)
        vb_right = plot_pg.ViewBox()
        plot_time.scene().addItem(vb_right)
        plot_time.getAxis('right').linkToView(vb_right)
        vb_right.setXLink(plot_time)
        
        # Update viewbox on resize
        def updateViews():
            vb_right.setGeometry(plot_time.getViewBox().sceneBoundingRect())
            vb_right.linkedViewChanged(plot_time.getViewBox(), vb_right.XAxis)
        plot_time.getViewBox().sigResized.connect(updateViews)
        
        # Right plot: scatter
        plot_scatter = plot_layout.addPlot(row=0, col=1, title="Phase Space")
        plot_scatter.setLabel('bottom', 'Rp (Ohms)')
        plot_scatter.setLabel('left', 'Inductance (uH)')
        
        # Configure scatter axes to show across full plot area
        plot_item = plot_scatter
        plot_item.layout.setContentsMargins(50, 10, 10, 50)
        plot_item.showAxis('top')
        plot_item.showAxis('right')
        
        # Create plot curves
        plot_curve_time_l = plot_time.plot([], [], pen=plot_pg.mkPen('cyan', width=2))  # left axis for L
        plot_curve_time_rp = plot_pg.PlotCurveItem([], [], pen=plot_pg.mkPen('yellow', width=2))  # right axis for Rp
        vb_right.addItem(plot_curve_time_rp)
        plot_curve_scatter = plot_scatter.plot([], [], pen=None, symbol='o', symbolPen=None, symbolBrush=None, symbolSize=5)

        # Live raw-packet readout spanning both columns. Shows exactly what the
        # LDC1101 is sending each tick, INCLUDING NaN/timeout packets that the
        # plots silently drop, so a dead/misconfigured sensor is visible at a glance.
        plot_readout = plot_layout.addLabel(
            "LDC1101: waiting for first packet...",
            row=1, col=0, colspan=2,
            justify='left', size='11pt'
        )

        # ----- Wrap the plots in a container that also holds a CLI input row -----
        # Lets the operator type Platform CLI commands (status, q 15, calibrate,
        # save al, ...) straight from the plot window. Commands are queued onto
        # joystick.pending_commands and the main loop sends them over the bridge
        # via serial_comm.send_command() (the [0xAB][len][ascii][0x55] frame).
        plot_container = plot_QtGui.QWidget()
        plot_container.setWindowTitle("BBot Telemetry + CLI")
        vbox = plot_QtGui.QVBoxLayout(plot_container)
        vbox.setContentsMargins(4, 4, 4, 4)
        vbox.addWidget(plot_layout)

        cmd_row = plot_QtGui.QHBoxLayout()
        cmd_label = plot_QtGui.QLabel("Platform CLI:")
        plot_cmd_input = plot_QtGui.QLineEdit()
        plot_cmd_input.setPlaceholderText("e.g.  status   help   q 15   calibrate   save al")
        plot_cmd_send = plot_QtGui.QPushButton("SEND")
        plot_cmd_status = plot_QtGui.QLabel("")
        plot_cmd_status.setStyleSheet("color: #888;")
        cmd_row.addWidget(cmd_label)
        cmd_row.addWidget(plot_cmd_input, 1)
        cmd_row.addWidget(plot_cmd_send)
        cmd_row.addWidget(plot_cmd_status)
        vbox.addLayout(cmd_row)

        def send_cli_command():
            text = plot_cmd_input.text().strip()
            if not text:
                return
            if joystick is not None:
                # Drained next loop tick by serial_comm.send_command(); same path
                # as the stick-click calibrate/rotate bindings.
                joystick.pending_commands.append(text)
                plot_cmd_status.setText(f"sent: {text}")
                print(f"\n[CLI] queued -> Platform: {text!r}")
            else:
                plot_cmd_status.setText("no link")
            plot_cmd_input.clear()
            plot_cmd_input.setFocus()

        plot_cmd_send.clicked.connect(send_cli_command)
        plot_cmd_input.returnPressed.connect(send_cli_command)  # Enter also sends

        plot_container.resize(1200, 680)
        plot_container.show()
        plot_ready = True
        print("plot windows initialized successfully")
    except Exception as e:
        print(f"plot window setup failed: {e}")
        plot_ready = False

class SerialComm:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=0.001)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.read_buffer = b''
        time.sleep(1)
        print(f"Serial connected at {baudrate} baud")
    
    def send_motor_command(self, left, right):
        """Send framed motor command: [START][4 data][END]"""
        left = max(min(int(left), 32767), -32768)
        right = max(min(int(right), 32767), -32768)
        
        packet = bytearray(6)
        packet[0] = PKT_START
        packet[1:3] = struct.pack('>h', left)   # Big-endian
        packet[3:5] = struct.pack('>h', right)
        packet[5] = PKT_END
        
        self.ser.write(packet)

    def send_command(self, text):
        """Send an ASCII CLI command to the Platform: [0xAB][len][ascii][0x55].

        The Platform feeds it into the same dispatcher as its USB serial CLI, so
        any CLI command works here (e.g. 'calibrate', 'rotated on', 'save al')."""
        payload = text.encode('ascii', 'ignore')[:95]
        packet = bytearray([CMD_START, len(payload)])
        packet += payload
        packet.append(PKT_END)
        self.ser.write(packet)

    def read_ldc_packet(self, timeout_ms=8):
        """Wait for and read a complete LDC packet"""
        end_time = time.time() + (timeout_ms / 1000.0)
        
        while time.time() < end_time:
            # Add incoming data to buffer
            if self.ser.in_waiting:
                self.read_buffer += self.ser.read(self.ser.in_waiting)
            
            # Look for start marker
            start_idx = self.read_buffer.find(bytes([PKT_START]))
            if start_idx >= 0 and len(self.read_buffer) >= start_idx + RESP_LEN:
                packet = self.read_buffer[start_idx:start_idx + RESP_LEN]

                # Check end marker
                if packet[RESP_LEN - 1] == PKT_END:
                    # Payload is everything between START and END.
                    data = packet[1:RESP_LEN - 1]
                    try:
                        ts, = struct.unpack('<Q', data[0:8])
                        motors = struct.unpack('<hh', data[8:12])
                        rp, = struct.unpack('<f', data[12:16])
                        l,  = struct.unpack('<f', data[16:20])
                        flags = data[20]
                        crack_size, = struct.unpack('<f', data[21:25])

                        # The END marker (checked above) is enough to trust this
                        # frame, so return it even when Rp/L are NaN. NaN means the
                        # firmware's LDC read failed (NO_SENSOR_OSC / DRDYB timeout /
                        # dead SPI); the caller surfaces it in the readout instead of
                        # us silently dropping it and masking a dead sensor.
                        self.read_buffer = self.read_buffer[start_idx + RESP_LEN:]
                        return ts, motors, rp, l, flags, crack_size
                    except:
                        # Bad data, remove first byte and try again
                        self.read_buffer = self.read_buffer[1:]
                else:
                    # Bad end marker, remove first byte
                    self.read_buffer = self.read_buffer[1:]
            elif start_idx < 0 and len(self.read_buffer) > 64:
                # No marker found, keep buffer small
                self.read_buffer = self.read_buffer[-64:]
            
            # Small delay to prevent CPU spinning
            time.sleep(0.0001)
        
        return None  # Timeout
    
    def clear_buffer(self):
        """Clear any stale data"""
        self.ser.reset_input_buffer()
        self.read_buffer = b''
    
    def close(self):
        self.ser.close()

# ===== HDF5 RECORDER (LDC + MOTORS) =====
class HDF5Recorder:
    def __init__(self, session_folder):
        self.h5_path = os.path.join(session_folder, "ldc_data.h5")
        self.h5_file = h5py.File(self.h5_path, 'w')
        
        # ===== DATASETS =====
        self.ts_dset = self.h5_file.create_dataset(
            'timestamp', (0,), maxshape=(None,),
            dtype='uint64', chunks=True, compression='gzip'
        )

        self.motor_dset = self.h5_file.create_dataset(
            'motors', (0, 2), maxshape=(None, 2),
            dtype='int16', chunks=True, compression='gzip'
        )

        self.rp_dset = self.h5_file.create_dataset(
            'rp_ohms', (0,), maxshape=(None,),
            dtype='float32', chunks=True, compression='gzip'
        )

        self.l_dset = self.h5_file.create_dataset(
            'inductance_uH', (0,), maxshape=(None,),
            dtype='float32', chunks=True, compression='gzip'
        )

        self.crack_dset = self.h5_file.create_dataset(
            'crack_detected', (0,), maxshape=(None,),
            dtype='bool', chunks=True, compression='gzip'
        )

        self.crack_size_dset = self.h5_file.create_dataset(
            'crack_size_thou', (0,), maxshape=(None,),
            dtype='float32', chunks=True, compression='gzip'
        )

        self.frame_flag_dset = self.h5_file.create_dataset(
            'frame_captured', (0,), maxshape=(None,),
            dtype='bool', chunks=True, compression='gzip'
        )

        self.frame_num_dset = self.h5_file.create_dataset(
            'frame_number', (0,), maxshape=(None,),
            dtype='int32', chunks=True, compression='gzip'
        )
        
        # ===== METADATA =====
        self.h5_file.attrs['start_time'] = time.time()
        self.h5_file.attrs['sample_rate_hz'] = HERTZ
        self.h5_file.attrs['camera_fps'] = fps
        
        self.sample_count = 0
    
    def append(self, ts, motors, rp, l, crack_detected, crack_size, frame_captured, frame_num):
        idx = self.sample_count

        # Resize
        self.ts_dset.resize((idx + 1,))
        self.motor_dset.resize((idx + 1, 2))
        self.rp_dset.resize((idx + 1,))
        self.l_dset.resize((idx + 1,))
        self.crack_dset.resize((idx + 1,))
        self.crack_size_dset.resize((idx + 1,))
        self.frame_flag_dset.resize((idx + 1,))
        self.frame_num_dset.resize((idx + 1,))

        # Write
        self.ts_dset[idx] = ts
        self.motor_dset[idx] = motors
        self.rp_dset[idx] = rp
        self.l_dset[idx] = l
        self.crack_dset[idx] = crack_detected
        self.crack_size_dset[idx] = crack_size
        self.frame_flag_dset[idx] = frame_captured
        self.frame_num_dset[idx] = frame_num if frame_captured else -1

        self.sample_count += 1
    
    def close(self):
        self.h5_file.attrs['end_time'] = time.time()
        self.h5_file.attrs['sample_count'] = self.sample_count
        self.h5_file.close()
        print(f"\nHDF5 saved: {self.h5_path} ({self.sample_count} samples)")
# ===== JOYSTICK CONTROL =====
class JoystickControl:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            raise Exception("No joystick found")
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Controller: {self.joystick.get_name()}")
        
        # Motor state
        self.left_pwm = 0
        self.right_pwm = 0
        self.power = 50
        self.polarity = 1
        self.recording = False
        self.running = True
        self.smoothing_window = 10

        # On-device CLI commands queued by button presses, drained in the main
        # loop and sent to the Platform. `rotated` mirrors the device's rotation
        # state (kept in sync from telemetry flags) so the toggle stays correct.
        self.pending_commands = []
        self.rotated = False

        # For labeling materials on the scatter plot
        self.material_label_count = 0
        self.material_labels = []
        
        # Recording objects
        self.h5_recorder = None
        self.session_folder = None
    
    def update(self):
        """Process joystick events and calculate motor values"""
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                self.handle_button(event)
        
        pygame.event.pump()
        
        # Tank drive
        threshold = 0.1
        left_y = self.joystick.get_axis(1) * self.polarity
        right_y = self.joystick.get_axis(3) * self.polarity
        
        # Deadzone
        if abs(left_y) < threshold:
            left_y = 0
        if abs(right_y) < threshold:
            right_y = 0
        
        # Scale to PWM
        self.left_pwm = int(left_y * 255 * self.power / 100)
        self.right_pwm = int(right_y * 255 * self.power / 100)
        
        return self.left_pwm, self.right_pwm
    
    def handle_button(self, event):
        global video_out, current_video_path, plot_l, plot_rp, plot_scatter, plot_pg, plot_time, vb_right
        
        # LB - decrease power
        if event.button == 4:
            self.power = max(0, self.power - 10)
            print(f"\nPower: {self.power}%")
        
        # RB - increase power
        elif event.button == 5:
            self.power = min(100, self.power + 10)
            print(f"\nPower: {self.power}%")
        
        # B - place material marker
        elif event.button == 2:
            # Place a label on the right plot at the most recent x-y coordinates
            self.material_label_count += 1
            label_text = f"Material\n {self.material_label_count}"
            # Place label at most recent point on scatter plot
            if plot_scatter is not None and len(plot_rp) > 0 and len(plot_l) > 0:
                x = plot_rp[-1]
                y = plot_l[-1]
                label = plot_pg.TextItem(label_text, anchor=(0.5, 0.5), color=(255,255,255), fill=(0,0,0,0))
                label.setPos(x, y)
                plot_scatter.addItem(label)
                self.material_labels.append(label)
                print(f"\nAdded label '{label_text}' at ({x:.2f}, {y:.2f})")
            else:
                print("\nNo data to label on scatter plot.")
        
        # A - toggle recording
        elif event.button == 3:
            self.recording = not self.recording
            if self.recording:
                self.start_recording()
            else:
                self.stop_recording()

        # X - reset the whole plot
        elif event.button == 0:
            # Remove all material labels from scatter plot
            for label in self.material_labels:
                if plot_scatter is not None:
                    plot_scatter.removeItem(label)
            self.material_labels = []
            self.material_label_count = 0
            plot_l = []
            plot_rp = []
            # Reset plot limits to default
            if plot_time is not None:
                plot_time.setXRange(0, 100)
                plot_time.setYRange(0, 100)
            if vb_right is not None:
                vb_right.setYRange(0, 100)
            print("\nPlot reset and limits reset")

        # Left stick click - run on-device PCA calibration
        elif event.button == 9:
            self.pending_commands.append("calibrate")
            print("\n[cmd] calibrate")

        # Right stick click - toggle on-device rotation (calibrated flatten)
        elif event.button == 10:
            self.rotated = not self.rotated
            self.pending_commands.append("rotated on" if self.rotated else "rotated off")
            print(f"\n[cmd] rotated {'on' if self.rotated else 'off'}")

        # Left trigger - decrease smoothing window
        elif event.button == 6:
            self.smoothing_window = max(1, self.smoothing_window - 1)
            print(f"\nSmoothing window: {self.smoothing_window}")

        # Right trigger - increase smoothing window
        elif event.button == 7:
            self.smoothing_window += 1
            print(f"\nSmoothing window: {self.smoothing_window}")

        # Back - quit
        elif event.button == 8:
            self.running = False
            print("\nButton 8")
        
    
    def start_recording(self):
        global video_out, current_video_path
        
        """Create new session folder and start recording"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_folder = os.path.join(RECORDINGS_FOLDER, f"session_{timestamp}")
        os.makedirs(self.session_folder, exist_ok=True)
        
        # Start HDF5 recording
        self.h5_recorder = HDF5Recorder(self.session_folder)
        
        # Start video recording
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # type: ignore[attr-defined]  # valid at runtime; missing from cv2 stubs
        current_video_path = get_new_filename(self.session_folder, "video", ".mp4")
        video_out = cv2.VideoWriter(current_video_path, fourcc, fps, (width, height))
        
        print(f"\nRECORDING started - {os.path.basename(self.session_folder)}")
        print(f"   Video: {os.path.basename(current_video_path)}")
    
    def stop_recording(self):
        global video_out, current_video_path
        
        """Stop recording and close files"""
        # Stop HDF5 recording
        if self.h5_recorder:
            self.h5_recorder.close()
            self.h5_recorder = None
        
        # Stop video recording
        if video_out is not None:
            video_out.release()
            video_out = None
            if current_video_path and os.path.exists(current_video_path):
                size = os.path.getsize(current_video_path) / (1024 * 1024)
                print(f"   Video saved: {os.path.basename(current_video_path)} ({size:.1f} MB)")
            current_video_path = None
        
        print(f"\nRECORDING stopped")
        self.session_folder = None
    
    def log_data(self, ts, motors, rp, l, crack_detected, crack_size, frame_info):
        """Log one sample if recording"""
        if self.recording and self.h5_recorder:
            frame_captured = frame_info is not None
            frame_num = frame_info[1] if frame_info else -1

            self.h5_recorder.append(
                ts, motors, rp, l,
                crack_detected, crack_size,
                frame_captured, frame_num
            )

# ===== MAIN LOOP =====
def main():
    global video_out, current_video_path, plot_l, plot_rp, plot_ready, plot_curve, plot_QtGui
    
    print("=" * 60)
    print(f"BBot Control - {HERTZ}Hz Request-Response Mode")
    print("=" * 60)
    print("Controls:")
    print("  Left Stick: Left motor      Right Stick: Right motor")
    print("  LB/RB: Power -/+            LT/RT: Smoothing window -/+")
    print("  A: Start/Stop recording    B: Place material marker    X: Reset plot")
    print("  L-Stick click: Calibrate (on-device)")
    print("  R-Stick click: Toggle rotation (on-device)")
    print("  Back/View: Quit")
    print("=" * 60)
    
    # Initialize
    try:
        joystick = JoystickControl()
    except Exception as e:
        print(f"Joystick error: {e}")
        return
    
    serial_comm = SerialComm(PORT, BAUDRATE)

    # Bring up plots immediately; data will fill in as packets arrive.
    init_plot_window(joystick)
    
    # Clear any stale data
    serial_comm.clear_buffer()
    time.sleep(0.5)
    
    # Timing
    last_status = time.time()
    loop_count = 0
    response_success = 0
    response_timeout = 0
    response_nan = 0  # packets received but with NaN Rp/L (sensor read failed)
    frame_counter = 0
    
    try:
        while joystick.running:
            cycle_start = time.time()
            frame_info = None
            frame_counter += 1
            
            # ===== 1. Get joystick input =====
            left, right = joystick.update()
            
            # ===== 1b. Send any queued on-device CLI commands (calibrate / rotate) =====
            for cmd in joystick.pending_commands:
                serial_comm.send_command(cmd)
            joystick.pending_commands.clear()

            # ===== 2. Send motor command =====
            serial_comm.send_motor_command(left*-1, right)
            
            # ===== 3. Simple camera grab =====
            ret, frame = cap.read()
            if ret:
                # Show the frame
                cv2.imshow('Camera', frame)
                cv2.waitKey(1)
                
                # Create frame_info for logging (timestamp, frame_num)
                frame_info = (time.time(), frame_counter)
                
                # Save to video if recording
                if joystick.recording and video_out is not None:
                    video_out.write(frame)
            
            # ===== 4. Wait for and read LDC response =====
            timeout_ms = int(LOOP_INTERVAL * 1000 * 0.8)
            ldc_packet = serial_comm.read_ldc_packet(timeout_ms)
            
            if ldc_packet:
                ts, motors, rp, l, flags, crack_size = ldc_packet

                # ----- Live raw-packet readout (shows NaN packets too) -----
                # Updated here, BEFORE the finite-gate below, so a dead or
                # misconfigured LDC1101 (which sends NaN) is still visible.
                if plot_ready and plot_readout is not None:
                    rp_finite = math.isfinite(rp)
                    l_finite = math.isfinite(l)
                    sensor_ok = rp_finite and l_finite
                    cal = bool(flags & FLAG_ROTATED)
                    crk = bool(flags & FLAG_CRACK)
                    color = '#33ff33' if sensor_ok else '#ff5050'
                    rp_str = f"{rp:.2f}" if rp_finite else "NaN"
                    l_str = f"{l:.3f}" if l_finite else "NaN"
                    cs_str = f"{crack_size:.1f}" if math.isfinite(crack_size) else "NaN"
                    plot_readout.setText(
                        f"<span style='color:{color}'>"
                        f"Rp={rp_str} &#937;&nbsp;&nbsp; L={l_str} uH</span>"
                        f"&nbsp;&nbsp;|&nbsp;&nbsp; flags=0x{flags:02X} "
                        f"(calibrated={int(cal)}, crack={int(crk)})"
                        f"&nbsp;&nbsp;|&nbsp;&nbsp; crack_size={cs_str} thou"
                        f"&nbsp;&nbsp;|&nbsp;&nbsp; good={response_success} "
                        f"nan={response_nan} timeouts={response_timeout}",
                        size='11pt'
                    )
                    plot_QtGui.QApplication.processEvents()

                if not (math.isfinite(rp) and math.isfinite(l)):
                    # Packet arrived but the sensor read failed (NaN). Count it
                    # separately from a true link timeout so the readout can tell
                    # "dead sensor" from "no packets".
                    response_nan += 1
                    continue

                # Finite, usable sample.
                response_success += 1
                if not math.isfinite(crack_size):
                    crack_size = 0.0
                crack_detected = bool(flags & FLAG_CRACK)
                device_calibrated = bool(flags & FLAG_ROTATED)
                # Keep the rotate-toggle in sync with the device's actual state.
                joystick.rotated = device_calibrated

                if plot_ready:
                    plot_l.append(l)
                    plot_rp.append(rp)
                    # Keep only last 10000 points
                    if len(plot_l) > 10000:
                        plot_l = plot_l[-10000:]
                        plot_rp = plot_rp[-10000:]
                    
                    # Compute moving average of last [window] points
                    # Use smoothing_window from joystick
                    window = getattr(joystick, 'smoothing_window', 25)
                    smoothed_l = []
                    smoothed_rp = []
                    for i in range(len(plot_l)):
                        start = max(0, i - window + 1)
                        segment_l = plot_l[start:i+1]
                        segment_rp = plot_rp[start:i+1]
                        avg_l = sum(segment_l) / len(segment_l)
                        avg_rp = sum(segment_rp) / len(segment_rp)
                        smoothed_l.append(avg_l)
                        smoothed_rp.append(avg_rp)

                    # Keep only finite points for plotting and ranges.
                    finite_points = [
                        (x, y) for x, y in zip(smoothed_rp, smoothed_l)
                        if math.isfinite(x) and math.isfinite(y)
                    ]
                    if not finite_points:
                        plot_QtGui.QApplication.processEvents()
                        continue
                    smoothed_rp = [p[0] for p in finite_points]
                    smoothed_l = [p[1] for p in finite_points]
                    
                    # Create color gradient from red (old) to orange, with last 20 points bright white
                    num_points = len(smoothed_l)
                    colors = []
                    for i in range(num_points):
                        if i >= num_points - 20:
                            # Last 20 points: bright white
                            colors.append((255, 255, 255, 200))
                        else:
                            # Earlier points: gradient from red to orange
                            progress = i / max(1, num_points - 21)
                            r = 255
                            g = int(165 * progress)
                            b = 0
                            colors.append((r, g, b, 200))
                    
                    # Time series data
                    time_x = list(range(num_points))
                    
                    # Update plots
                    plot_curve_time_l.setData(time_x, smoothed_l)
                    plot_curve_time_rp.setData(time_x, smoothed_rp)
                    plot_curve_scatter.setData(smoothed_rp, smoothed_l, pen=None, symbol='o', symbolPen=None, symbolBrush=colors, symbolSize=5)
                    
                    # Update axis ranges
                    if num_points > 1:
                        # Scatter plot ranges
                        x_min, x_max = min(smoothed_rp), max(smoothed_rp)
                        y_min, y_max = min(smoothed_l), max(smoothed_l)
                        x_pad = max(0.01, (x_max - x_min) * 0.01) if x_max > x_min else 0.1
                        y_pad = max(0.1, (y_max - y_min) * 0.002) if y_max > y_min else 0.5
                        plot_scatter.setXRange(x_min - x_pad, x_max + x_pad)
                        plot_scatter.setYRange(y_min - y_pad, y_max + y_pad)
                        
                        # Time series ranges - separate axes
                        l_min, l_max = min(smoothed_l), max(smoothed_l)
                        rp_min, rp_max = min(smoothed_rp), max(smoothed_rp)
                        l_pad = max(0.01, (l_max - l_min) * 0.01) if l_max > l_min else 0.1
                        rp_pad = max(1, (rp_max - rp_min) * 0.01) if rp_max > rp_min else 5
                        plot_time.setYRange(l_min - l_pad, l_max + l_pad)
                        vb_right.setYRange(rp_min - rp_pad, rp_max + rp_pad)
                        plot_time.setXRange(0, num_points - 1)
                    
                    plot_QtGui.QApplication.processEvents()

                if crack_detected:
                    # Mark the detection on the scatter plot and announce it.
                    if plot_ready and len(plot_rp) > 0:
                        marker = plot_pg.TextItem("x", anchor=(0.5, 0.5), color=(255, 0, 0))
                        marker.setPos(plot_rp[-1], plot_l[-1])
                        plot_scatter.addItem(marker)
                    print(f"\n>>> CRACK detected: size~{crack_size:.1f} thou")

                print(f"\r{ts/1e6:.3f},{rp:.2f},{l:.2f}uH", end='')
                # Log if recording
                joystick.log_data(ts, motors, rp, l, crack_detected, crack_size, frame_info)
            else:
                response_timeout += 1
                if plot_ready and plot_readout is not None:
                    plot_readout.setText(
                        f"<span style='color:#ffaa00'>no telemetry packet "
                        f"(timeout)</span>&nbsp;&nbsp;|&nbsp;&nbsp; "
                        f"good={response_success} nan={response_nan} "
                        f"timeouts={response_timeout}",
                        size='11pt'
                    )
                    plot_QtGui.QApplication.processEvents()

            # ===== 5. Maintain loop timing =====
            loop_count += 1
            elapsed = time.time() - cycle_start
            sleep_time = max(0, LOOP_INTERVAL - elapsed)
            time.sleep(sleep_time)
            
            # ===== 6. Status display =====
            if time.time() - last_status > 0.5:
                if loop_count > 0:
                    success_rate = response_success / loop_count * 100
                else:
                    success_rate = 0
                
                # Camera status
                cam_status = "Cam Good" if ret else "Cam Bad"
                
                print(f"\rL={left:4d} R={right:4d} | "
                      f"Power: {joystick.power:2d}% | "
                      f"Response: {success_rate:3.0f}% | "
                      f"{cam_status} | "
                      f"{'Recording...' if joystick.recording else 'Ready'}      ", 
                      end='', flush=True)
                
                loop_count = 0
                response_success = 0
                response_timeout = 0
                response_nan = 0
                last_status = time.time()
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        # Clean shutdown
        if joystick.recording:
            joystick.stop_recording()
        
        serial_comm.send_motor_command(0, 0)
        serial_comm.close()
        cap.release()
        if video_out is not None:
            video_out.release()
        cv2.destroyAllWindows()
        pygame.quit()
        print("\nShutdown complete")

if __name__ == "__main__":
    main()
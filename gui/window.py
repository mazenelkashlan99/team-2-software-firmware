import cv2
import threading
import time
import numpy as np
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                               QLabel, QFormLayout, QGroupBox, QSizePolicy)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
import data
import StatusWidget


class Camera:
    def __init__(self) -> None:
        self.capture = cv2.VideoCapture(0)
        self.is_running = True
        self._frame = None
        self.empty_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self) -> None:
        while self.is_running:
            success, image = self.capture.read()
            if success:
                with self.lock:
                    self._frame = image
            time.sleep(0.01)

    @property
    def frame(self):
        with self.lock:
            return self._frame if self._frame is not None else self.empty_frame

    def release(self):
        self.is_running = False
        self.capture.release()


class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Telemetry Dashboard")
        self.resize(1150, 800)

        self.speed_data = data.SpeedData()
        self.battery_data = data.BatteryData()
        self.map_data = data.MapData()

        self.speed_widget = StatusWidget.SpeedWidget.get_widget(self.speed_data, self)
        self.battery_widget = StatusWidget.BatteryWidget.get_widget(self.battery_data, self)
        self.map_widget = StatusWidget.MapWidget.get_widget(self.map_data, self)

        self.battery_widget.setMinimumSize(150, 60)
        self.battery_widget.setMaximumHeight(80)

        self.speed_widget.setMinimumSize(350, 250)
        self.speed_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.speed_widget.setMaximumHeight(300)

        # Map widget sizing
        self.map_widget.setMinimumSize(250, 250)
        self.map_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self._central_widget = QWidget(self)
        self._main_layout = QVBoxLayout(self._central_widget)
        self._main_layout.setContentsMargins(15, 15, 15, 15)
        self._main_layout.setSpacing(20)

        self._camera_label = QLabel("Initializing Camera...")
        self._camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._camera_label.setFrameStyle(QLabel.Shape.Box | QLabel.Shadow.Plain)
        self._camera_label.setStyleSheet("background-color: black;")
        self._camera_label.setMinimumSize(640, 360)

        self.camera = Camera()
        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera_feed)
        self.camera_timer.start(33)

        self._bottom_layout = QHBoxLayout()

        # 1. Left: Battery and Speed
        self._left_layout = QVBoxLayout()
        self._left_layout.setSpacing(20)
        self._left_layout.addWidget(self.battery_widget, alignment=Qt.AlignmentFlag.AlignHCenter)
        self._left_layout.addWidget(self.speed_widget, alignment=Qt.AlignmentFlag.AlignHCenter)
        self._left_layout.addStretch()

        # 2. Center: 2D Map
        self._center_layout = QVBoxLayout()
        self._center_layout.addWidget(self.map_widget, alignment=Qt.AlignmentFlag.AlignCenter)

        # 3. Right: Telemetry
        self._right_layout = QVBoxLayout()
        self._text_group = QGroupBox("ROBOT TELEMETRY")
        self._text_layout = QFormLayout()
        self._text_layout.setVerticalSpacing(10)
        self._text_layout.setHorizontalSpacing(15)
        self._telemetry_labels = {}

        def add_telemetry_row(key_text, val_text):
            key_lbl = QLabel(key_text)
            val_lbl = QLabel(val_text)
            self._text_layout.addRow(key_lbl, val_lbl)
            self._telemetry_labels[key_text] = val_lbl

        add_telemetry_row("Robot Position:", "x: 0.00, y: 0.00")
        add_telemetry_row("Robot Heading:", "0.00 rad")
        add_telemetry_row("Robot Velocity:", "v: 0.00, w: 0.00")
        add_telemetry_row("Operating Mode:", "MANUAL")
        add_telemetry_row("Autonomous State:", "IDLE")
        add_telemetry_row("Arm Status:", "STOWED")
        add_telemetry_row("Battery Voltage:", "0.0 V")
        add_telemetry_row("Position Validity:", "False")
        add_telemetry_row("Comm Link Status:", "DISCONNECTED")
        add_telemetry_row("Robot Status:", "OFFLINE")
        add_telemetry_row("Mission Events:", "ev: NONE, detail: NONE")
        add_telemetry_row("System Logs:", "level: INFO, message: none")
        add_telemetry_row("Comm Latency:", "ping_id: 0, t_send: 0, RTT: 0 ms")
        add_telemetry_row("QR Detection:", "valid: False, raw: '', x: 0.0, y: 0.0")
        add_telemetry_row("Color Detection:", "N/A")

        self._text_group.setLayout(self._text_layout)
        self._right_layout.addWidget(self._text_group)
        self._right_layout.addStretch()

        # Add all three columns to the bottom layout
        self._bottom_layout.addLayout(self._left_layout, stretch=1)
        self._bottom_layout.addLayout(self._center_layout, stretch=1)
        self._bottom_layout.addLayout(self._right_layout, stretch=1)

        self._main_layout.addWidget(self._camera_label, stretch=3)
        self._main_layout.addLayout(self._bottom_layout, stretch=2)

        self.setCentralWidget(self._central_widget)
        self.show()

    def update_camera_feed(self):
        frame = self.camera.frame
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        bytes_per_line = ch * w

        qt_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        scaled_image = qt_img.scaled(self._camera_label.width(), self._camera_label.height(),
                                     Qt.AspectRatioMode.KeepAspectRatio,
                                     Qt.TransformationMode.FastTransformation)
        self._camera_label.setPixmap(QPixmap.fromImage(scaled_image))

    def closeEvent(self, event):
        self.camera_timer.stop()
        self.camera.release()
        super().closeEvent(event)

    def speed_changed(self, speed, angular_speed):
        self.speed_data.speed = speed
        self._telemetry_labels["Robot Velocity:"].setText(f"v: {speed:.2f}, w: {angular_speed:.2f}")

    def position_changed(self, x, y, theta):
        self.map_data.x = x
        self.map_data.y = y
        self.map_data.theta = theta
        self._telemetry_labels["Robot Position:"].setText(f"x: {x:.2f}, y: {y:.2f}")
        self._telemetry_labels["Robot Heading:"].setText(f"{theta:.2f} rad")

    def battery_changed(self, voltage):
        self.battery_data.level = (voltage / 5.0) * 100
        self._telemetry_labels["Battery Voltage:"].setText(f"{voltage:.2f} V")

    def update_operating_mode(self, mode):
        self._telemetry_labels["Operating Mode:"].setText(mode)

    def update_autonomous_state(self,  state):
        self._telemetry_labels["Autonomous State:"].setText(state)

    def update_position_validity(self, is_valid):
        self._telemetry_labels["Position Validity:"].setText(str(is_valid))

    def update_communication_link(self, status):
        if status:
            connect = "CONNECTED"
        else:
            connect = "DISCONNECTED"
        self._telemetry_labels["Comm Link Status:"].setText(connect)

    def update_robot_status(self, status):
        if status:
            connect = "ONLINE"
        else:
            connect = "OFFLINE"
        self._telemetry_labels["Robot Status:"].setText(connect)

    def update_mission_event(self, event):
        self._telemetry_labels["Mission Events:"].setText(event)

    def update_system_logs(self, status):
        self._telemetry_labels["System Logs:"].setText(status)

    def update_communication_latency(self, ping_id, t_send, rtt):
        self._telemetry_labels["Comm Latency:"].setText(f"ping_id: {ping_id}, t_send: {t_send}, RTT: {rtt} ms")

    def update_qr_detection(self, valid, raw, x, y):
        self._telemetry_labels["QR Detection:"].setText(f"valid: {valid}, raw: '{raw}', x: {x:.2f}, y: {y:.2f}")

    def update_color_detection(self,  color):
        self._telemetry_labels["Color Detection:"].setText(color)
import cv2
import threading
import time
import numpy as np
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                               QLabel, QFormLayout, QGroupBox, QSizePolicy,
                               QPushButton, QScrollArea)
from PySide6.QtCore import Qt, QTimer , Signal ,Slot
from PySide6.QtGui import QImage, QPixmap
import data
import StatusWidget
from rov_mqtt.mqtt_client import MQTTClient
import json
from qr_scanner import scan_qr
from rov_mqtt.controller import PS4Controller



class Camera:
    def __init__(self) -> None:
        self.capture = cv2.VideoCapture(
            "rtsp://192.168.1.2:554/stream",
            cv2.CAP_FFMPEG
        )
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
    mqtt_message_received = Signal(str,str)
    mqtt_connection_changed = Signal(bool)
    def __init__(self):

        super().__init__()
        self.setWindowTitle("Robot Telemetry Dashboard")
        screen = self.screen()
        available = screen.availableGeometry() if screen is not None else None
        if available is not None and available.isValid():
            # Leave a little room around the work area so Windows can account
            # for the title bar and taskbar without emitting setGeometry warnings.
            initial_width = min(1150, max(1, available.width() - 16))
            initial_height = min(760, max(1, available.height() - 16))
            minimum_width = min(900, initial_width)
            minimum_height = min(600, initial_height)
            self.setMinimumSize(minimum_width, minimum_height)
            self.resize(initial_width, initial_height)
            self.move(
                available.x() + (available.width() - initial_width) // 2,
                available.y() + (available.height() - initial_height) // 2,
            )
        else:
            self.setMinimumSize(800, 560)
            self.resize(1150, 760)

        self.speed_data = data.SpeedData()
        self.battery_data = data.BatteryData()
        self.map_data = data.MapData()
        self._pending_pings: dict[int, float] = {}
        self._ping_id = 0
        self._operating_mode = "ESTOP"
        self._robot_online = False
        self._broker_connected = False
        self._estop_latched = True
        self._estop_reset_requested = False

        self.speed_widget = StatusWidget.SpeedWidget.get_widget(self.speed_data, self)
        self.battery_widget = StatusWidget.BatteryWidget.get_widget(self.battery_data, self)
        self.map_widget = StatusWidget.MapWidget.get_widget(self.map_data, self)

        self.battery_widget.setMinimumSize(120, 48)
        self.battery_widget.setMaximumHeight(80)

        self.speed_widget.setMinimumSize(240, 180)
        self.speed_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.speed_widget.setMaximumHeight(260)

        # Map widget sizing
        self.map_widget.setMinimumSize(200, 180)
        self.map_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self._central_widget = QWidget(self)
        self._main_layout = QVBoxLayout(self._central_widget)
        self._main_layout.setContentsMargins(8, 8, 8, 8)
        self._main_layout.setSpacing(8)

        self._camera_label = QLabel("Initializing Camera...")
        self._camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._camera_label.setFrameStyle(QLabel.Shape.Box | QLabel.Shadow.Plain)
        self._camera_label.setStyleSheet("background-color: black;")
        self._camera_label.setMinimumSize(480, 240)
        self._camera_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )

        self.camera = Camera()
        self.qr_detector = cv2.QRCodeDetector()

        self.qr_visible = False
        self.last_seen_time = 0.0
        self.disappear_delay = 2.0

        self.qr_coordinates = []

        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera_feed)
        self.camera_timer.start(5)

        self._bottom_layout = QHBoxLayout()

        # 1. Left: Battery and Speed
        self._left_layout = QVBoxLayout()
        self._left_layout.setSpacing(8)
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
        self._text_layout.setVerticalSpacing(4)
        self._text_layout.setHorizontalSpacing(8)
        self._telemetry_labels = {}

        def add_telemetry_row(key_text, val_text):
            key_lbl = QLabel(key_text)
            val_lbl = QLabel(val_text)
            self._text_layout.addRow(key_lbl, val_lbl)
            self._telemetry_labels[key_text] = val_lbl

        add_telemetry_row("Robot Position:", "x: 0.00, y: 0.00")
        add_telemetry_row("Robot Heading:", "0.00 rad")
        add_telemetry_row("Robot Velocity:", "v: 0.00, w: 0.00")
        add_telemetry_row("Operating Mode:", "ESTOP")
        add_telemetry_row("Arm Status:", "STOWED")
        add_telemetry_row("Battery Voltage:", "0.0 V")
        add_telemetry_row("Position Validity:", "False")
        add_telemetry_row("Comm Link Status:", "DISCONNECTED")
        add_telemetry_row("Robot Status:", "OFFLINE")
        add_telemetry_row("Broker Connection:", "DISCONNECTED")
        add_telemetry_row("System Logs:", "lvl: INFO, msg: none")
        add_telemetry_row("Comm Latency:", "ping_id: 0, RTT: 0 ms")

        self._text_group.setLayout(self._text_layout)
        self._right_layout.addWidget(self._text_group)

        safety_controls = QGroupBox("SAFETY")
        safety_layout = QHBoxLayout(safety_controls)
        safety_layout.setContentsMargins(8, 6, 8, 6)
        safety_layout.setSpacing(8)
        self._estop_button = QPushButton("EMERGENCY STOP")
        self._estop_button.clicked.connect(self._send_estop)
        self._reset_estop_button = QPushButton("Reset ESTOP")
        self._reset_estop_button.clicked.connect(self._send_estop_reset)
        safety_layout.addWidget(self._estop_button)
        safety_layout.addWidget(self._reset_estop_button)
        self._right_layout.addStretch()

        # Add all three columns to the bottom layout
        self._bottom_layout.addLayout(self._left_layout, stretch=1)
        self._bottom_layout.addLayout(self._center_layout, stretch=1)
        self._bottom_layout.addLayout(self._right_layout, stretch=1)

        dashboard_content = QWidget()
        dashboard_layout = QVBoxLayout(dashboard_content)
        dashboard_layout.setContentsMargins(0, 0, 0, 0)
        dashboard_layout.setSpacing(8)
        dashboard_layout.addWidget(self._camera_label, stretch=3)
        dashboard_layout.addLayout(self._bottom_layout, stretch=2)

        dashboard_scroll = QScrollArea()
        dashboard_scroll.setWidgetResizable(True)
        dashboard_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        dashboard_scroll.setWidget(dashboard_content)
        self._main_layout.addWidget(dashboard_scroll, stretch=1)
        # Safety actions remain visible even when the dashboard needs scrolling.
        self._main_layout.addWidget(safety_controls, stretch=0)

        self.setCentralWidget(self._central_widget)
        self.mqtt_message_received.connect(self.handle_mqtt_message)
        self.mqtt_connection_changed.connect(self._update_broker_connection)
        self.mqtt_client=MQTTClient(client_id="rov_gui")
        self.mqtt_client.set_message_callback(self._on_mqtt_message_from_thread)
        self.mqtt_client.set_connection_callback(self._on_mqtt_connection_from_thread)
        self._update_safety_controls()
        self.mqtt_client.connect()
        self.ping_timer = QTimer(self)
        self.ping_timer.timeout.connect(self._send_ping)
        self.ping_timer.start(1000)


        self.controller = None
        self._previous_buttons = ()
        try:
            self.controller = PS4Controller()
            self._previous_buttons = self.controller.read_buttons()
        except RuntimeError as error:
            print(f"PS4 controller unavailable: {error}")

        self.controller_timer = QTimer(self)
        self.controller_timer.timeout.connect(self._poll_controller)
        self.controller_timer.start(50)

        self.show()


    def update_camera_feed(self):
        frame = self.camera.frame.copy()

        results, points = scan_qr(frame, self.qr_detector)
        current_time = time.time()

        if points is not None:
            points = points.astype(int)

            for qr_points in points:
                for i in range(4):
                    p1 = tuple(qr_points[i])
                    p2 = tuple(qr_points[(i + 1) % 4])

                    cv2.line(frame, p1, p2, (0, 0, 255), 3)

        if results:
            self.last_seen_time = current_time

            if not self.qr_visible:
                for data, x, y in results:
                    self.qr_coordinates.append((x, y))

                    print("New QR box detected!")
                    print(f"X: {x} m")
                    print(f"Y: {y} m")
                    print(
                        f"Total boxes scanned: "
                        f"{len(self.qr_coordinates)}"
                    )

                self.qr_visible = True

            for _, x, y in results:
                cv2.putText(
                    frame,
                    f"X: {x:.2f} m, Y: {y:.2f} m",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )

        elif (
            self.qr_visible
            and current_time - self.last_seen_time >= self.disappear_delay
        ):
            self.qr_visible = False

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        h, w, ch = frame.shape
        bytes_per_line = ch * w

        qt_img = QImage(
            frame.data,
            w,
            h,
            bytes_per_line,
            QImage.Format.Format_RGB888
        )

        label_width = self._camera_label.width()
        label_height = self._camera_label.height()

        scaled_image = qt_img.scaled(
            label_width,
            label_height,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )

        x = (scaled_image.width() - label_width) // 2
        y = (scaled_image.height() - label_height) // 2

        scaled_image = scaled_image.copy(
            x, y, label_width, label_height
        )

        self._camera_label.setPixmap(QPixmap.fromImage(scaled_image))

    def _on_mqtt_message_from_thread(self,topic,payload):
        self.mqtt_message_received.emit(topic,payload)

    def _on_mqtt_connection_from_thread(self, connected):
        self.mqtt_connection_changed.emit(connected)

    @Slot(bool)
    def _update_broker_connection(self, connected):
        self._broker_connected = connected
        if not connected:
            self._robot_online = False
            self._estop_latched = True
            self._estop_reset_requested = False
            self._telemetry_labels["Robot Status:"].setText("OFFLINE")
        self._telemetry_labels["Broker Connection:"].setText(
            "CONNECTED" if connected else "DISCONNECTED"
        )
        self._update_safety_controls()

    def _update_safety_controls(self):
        self._estop_button.setEnabled(self._broker_connected)
        self._reset_estop_button.setEnabled(
            self._broker_connected
            and self._robot_online
            and self._operating_mode == "ESTOP"
            and not self._estop_reset_requested
        )

    def _send_estop(self):
        print("[GUI] ESTOP handler invoked")
        self._estop_latched = True
        self._estop_reset_requested = False
        self._update_safety_controls()
        if not self._broker_connected:
            print("[GUI] ESTOP remains latched locally; broker is disconnected")
            return

        try:
            result = self.mqtt_client.publish_estop()
        except Exception as error:
            print(f"[GUI] ESTOP publish failed: {error}")
            return

        if result is not None and result.rc != 0:
            print(f"[GUI] ESTOP publish rejected (rc={result.rc})")
        else:
            print("[GUI] ESTOP command queued; waiting for ESTOP mode confirmation")

    def _send_estop_reset(self):
        print("[GUI] Reset ESTOP handler invoked")
        if (
            self._broker_connected
            and self._robot_online
            and self._operating_mode == "ESTOP"
            and not self._estop_reset_requested
        ):
            self._estop_reset_requested = True
            try:
                result = self.mqtt_client.publish_estop_reset()
            except Exception as error:
                self._estop_reset_requested = False
                print(f"[GUI] Reset ESTOP request could not be published: {error}")
                self._update_safety_controls()
                return

            if result is not None and result.rc != 0:
                self._estop_reset_requested = False
                print(f"[GUI] Reset ESTOP publish was rejected (rc={result.rc})")
            else:
                print("[GUI] Reset ESTOP request sent; waiting for MANUAL mode confirmation")
            self._update_safety_controls()
        else:
            print(
                "[GUI] Reset ESTOP request not sent: broker/robot/mode "
                "preconditions failed or reset is already pending"
            )

    @Slot(str,str)
    def handle_mqtt_message(self,topic,payload):
        if topic == "robot/status/online":
            status = payload.strip().lower()
            if status == "online":
                self.update_robot_status(True)
            elif status == "offline":
                self.update_robot_status(False)
            else:
                print(f"Invalid online status payload: {payload!r}")
            return

        try:
            message = json.loads(payload)
        except json.JSONDecodeError:
            print(f"Invalid Json received on {topic}")
            return
        if not isinstance(message, dict):
            print(f"Expected a JSON object on {topic}")
            return

        if topic == "robot/state/mode":
            mode = message.get("mode")
            if not self._has_valid_envelope(message):
                print("Invalid protocol envelope received on robot/state/mode")
            elif isinstance(mode, str) and mode in {"MANUAL", "ESTOP"}:
                self.update_operating_mode(mode)

        elif topic == "robot/state/pong":
            ping_id = message.get("ping_id")
            if isinstance(ping_id, int) and not isinstance(ping_id, bool):
                sent_at = self._pending_pings.pop(ping_id, None)
                if sent_at is not None:
                    rtt = (time.monotonic() - sent_at) * 1000
                    self.update_communication_latency(
                        ping_id,
                        round(rtt, 2),
                    )

        elif topic == "robot/log":
            level = message.get("lvl", "INFO")
            if not isinstance(level, str) or level not in {"DEBUG", "INFO", "WARN", "ERROR"}:
                level = "INFO"
            log_message = message.get("msg", "")
            self.update_system_logs(f"{level}: {log_message}")

        elif topic == "robot/telemetry":
            speed = self._number_or_default(message.get("v"), 0.0)
            angular_speed = self._number_or_default(message.get("w"), 0.0)
            self.speed_changed(
                speed,
                angular_speed,
            )
            self.position_changed(
                self._number_or_default(message.get("x"), 0.0),
                self._number_or_default(message.get("y"), 0.0),
                self._number_or_default(message.get("theta"), 0.0),
            )
            self.update_arm_status(message.get("arm","UNKNOWN"))
            self.update_position_validity(message.get("pose_valid") is True)
            self.update_communication_link(message.get("link_ok") is True)
            battery_voltage = self._number_or_default(message.get("batt_v"), 0.0)
            self.battery_changed(battery_voltage)
            self._telemetry_labels["Battery Voltage:"].setText(
                f"{battery_voltage:.1f} V"
            )
    def _send_ping(self):
        self._ping_id += 1
        if len(self._pending_pings) >= 128:
            oldest_ping_id = next(iter(self._pending_pings))
            del self._pending_pings[oldest_ping_id]
        self._pending_pings[self._ping_id] = time.monotonic()
        self.mqtt_client.publish_ping(
            ping_id=self._ping_id,
        )

    @staticmethod
    def _number_or_default(value, default):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return default
        try:
            value = float(value)
        except (OverflowError, ValueError):
            return default
        return value if np.isfinite(value) else default

    @staticmethod
    def _has_valid_envelope(message):
        version = message.get("ver")
        session_id = message.get("sid")
        sequence = message.get("seq")
        return (
            isinstance(version, int)
            and not isinstance(version, bool)
            and version == 1
            and isinstance(session_id, str)
            and 1 <= len(session_id) <= 15
            and isinstance(sequence, int)
            and not isinstance(sequence, bool)
            and sequence >= 1
        )

    def closeEvent(self, event):
        self.camera_timer.stop()
        self.ping_timer.stop()
        self.camera.release()
        self.controller_timer.stop()
        if self.controller is not None:
            self.controller.close()
        self.mqtt_client.disconnect()
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
        if not isinstance(mode, str) or mode not in {"ESTOP", "MANUAL"}:
            return

        self._operating_mode = mode
        if mode == "ESTOP":
            # Retained/repeated ESTOP reports can arrive while the reset is
            # pending. Keep the request pending until MANUAL is confirmed.
            self._estop_latched = True
        elif self._estop_reset_requested and self._broker_connected and self._robot_online:
            self._estop_reset_requested = False
            self._estop_latched = False
        self._telemetry_labels["Operating Mode:"].setText(mode)
        self._update_safety_controls()
        print(
            f"[GUI] Mode confirmation received: {mode}; "
            f"ESTOP latched={self._estop_latched}, "
            f"reset pending={self._estop_reset_requested}"
        )

    def update_position_validity(self, is_valid):
        self._telemetry_labels["Position Validity:"].setText(str(is_valid))

    def update_communication_link(self, status):
        if status:
            connect = "CONNECTED"
        else:
            connect = "DISCONNECTED"
        self._telemetry_labels["Comm Link Status:"].setText(connect)

    def update_robot_status(self, status):
        self._robot_online = bool(status)
        if not self._robot_online:
            self._estop_latched = True
            self._estop_reset_requested = False
        if status:
            connect = "ONLINE"
        else:
            connect = "OFFLINE"
        self._telemetry_labels["Robot Status:"].setText(connect)
        self._update_safety_controls()

    def update_system_logs(self, status):
        self._telemetry_labels["System Logs:"].setText(status)

    def update_communication_latency(self, ping_id, rtt):
        self._telemetry_labels["Comm Latency:"].setText(
            f"ping_id: {ping_id}, RTT: {rtt} ms"
        )

    def update_arm_status(self,status):
        self._telemetry_labels["Arm Status:"].setText(
            status if isinstance(status, str) else "UNKNOWN"
        )


    def _poll_controller(self):
        if self.controller is None:
            return
        try:
            buttons = self.controller.read_buttons()
        except(RuntimeError,IndexError) as error :
            print(f"Controller button read error: {error}")
            self._estop_latched = True
            self._estop_reset_requested = False
            self._previous_buttons = ()
            self._update_safety_controls()
            return

        previous = self._previous_buttons
        self._previous_buttons = buttons
        button_names = {
            0: "Cross/PICK",
            1: "Circle/DROP",
            2: "Square/STOW",
            3: "Triangle/Reset ESTOP",
            5: "PS",
            6: "OPTIONS/ESTOP",
        }
        for button_index in range(max(len(previous), len(buttons))):
            was_pressed = button_index < len(previous) and previous[button_index]
            is_pressed = button_index < len(buttons) and buttons[button_index]
            if is_pressed != was_pressed:
                transition = "pressed" if is_pressed else "released"
                name = button_names.get(button_index, "unmapped")
                print(f"[Controller] button {button_index} ({name}) {transition}")

        option_pressed = len(buttons) > 6 and buttons[6]
        option_was_pressed = len(previous) > 6 and previous[6]
        if option_pressed and not option_was_pressed:
            print("[GUI] OPTIONS button 6 press edge; invoking ESTOP handler")
            self._send_estop()

        triangle_pressed = len(buttons) > 3 and buttons[3]
        triangle_was_pressed = len(previous) > 3 and previous[3]
        if triangle_pressed and not triangle_was_pressed:
            print("[GUI] Triangle button 3 press edge; invoking Reset ESTOP handler")
            # If both safety buttons are pressed together, ESTOP takes priority.
            if not option_pressed:
                self._send_estop_reset()

        # _previous_buttons is updated before handler calls and axis reads so
        # every poll advances the edge detector, including failure paths.

        try:
            thr, turn = self.controller.read_drive()
        except (RuntimeError, IndexError) as error:
            print(f"Controller drive read error: {error}")
            self._estop_latched = True
            self._estop_reset_requested = False
            self._update_safety_controls()
            return

        can_control =( self._broker_connected
                      and self._robot_online
                      and self._operating_mode =="MANUAL"
                      and not self._estop_latched)

        if can_control:
            self.mqtt_client.publish_drive(thr,turn)
            actions = {0: "PICK", 1: "DROP", 2: "STOW"}
            for button, action in actions.items():
                currently_pressed = button < len(buttons) and buttons[button]
                previously_pressed = button < len(previous) and previous[button]
                if currently_pressed and not previously_pressed:
                    self.mqtt_client.publish_arm(action)
                    print(f"Arm Action requested: {action}")

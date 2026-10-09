from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                               QLabel, QFormLayout, QGroupBox, QSizePolicy)
from PySide6.QtCore import Qt
import data
import StatusWidget


class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Telemetry Dashboard")
        self.resize(1150, 600)

        self.speed_data = data.SpeedData()
        self.battery_data = data.BatteryData()

        self.speed_widget = StatusWidget.SpeedWidget.get_widget(self.speed_data, self)
        self.battery_widget = StatusWidget.BatteryWidget.get_widget(self.battery_data, self)


        self.battery_widget.setFixedSize(150, 60)
        self.speed_widget.setFixedSize(350, 250)

        self._central_widget = QWidget(self)
        self._main_layout = QHBoxLayout(self._central_widget)
        self._main_layout.setContentsMargins(15, 15, 15, 15)
        self._main_layout.setSpacing(20)

        self._left_layout = QVBoxLayout()
        self._left_layout.setSpacing(20)

        self._left_layout.addWidget(self.battery_widget,
                                    alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter)
        self._left_layout.addWidget(self.speed_widget,
                                    alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter)
        self._left_layout.addStretch()

        self._camera_label = QLabel("Camera Feed Placeholder\n(NO SIGNAL)")
        self._camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._camera_label.setFrameStyle(QLabel.Shape.Box | QLabel.Shadow.Plain)
        self._camera_label.setLineWidth(1)
        self._camera_label.setMinimumSize(450, 350)

        self._right_layout = QVBoxLayout()

        self._text_group = QGroupBox("ROBOT TELEMETRY")
        self._text_layout = QFormLayout()
        self._text_layout.setVerticalSpacing(10)
        self._text_layout.setHorizontalSpacing(15)

        def add_telemetry_row(key_text, val_text):
            key_lbl = QLabel(key_text)
            val_lbl = QLabel(val_text)
            self._text_layout.addRow(key_lbl, val_lbl)

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

        self._right_layout.addWidget(self._text_group, alignment=Qt.AlignmentFlag.AlignTop)
        self._right_layout.addStretch()

        self._main_layout.addLayout(self._left_layout, stretch=2)
        self._main_layout.addWidget(self._camera_label, stretch=4)
        self._main_layout.addLayout(self._right_layout, stretch=3)

        self.setCentralWidget(self._central_widget)
        self.show()
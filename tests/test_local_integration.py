"""End-to-end checks against a local Mosquitto broker on localhost:1883."""

import os
import socket
import subprocess
import sys
import threading
import time
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "gui"))

import numpy as np
import paho.mqtt.client as mqtt
from PySide6.QtWidgets import QApplication

import StatusWidget
import window as window_module
from window import Window


def broker_is_available():
    try:
        with socket.create_connection(("localhost", 1883), timeout=0.5):
            return True
    except OSError:
        return False


class CameraStub:
    def __init__(self):
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)

    def release(self):
        pass


class ControllerStub:
    def __init__(self):
        self.buttons = (False,) * 7

    def read_drive(self):
        return 0.4, -0.2

    def read_buttons(self):
        return self.buttons

    def close(self):
        pass


class HeartbeatProbe:
    """Wait for an actual fake-firmware heartbeat before testing startup order."""

    def __init__(self):
        self.received = threading.Event()
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"fake_firmware_probe_{uuid.uuid4().hex[:8]}",
        )
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect("localhost", 1883, keepalive=10)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            client.subscribe("robot/hb/robot", qos=0)

    def _on_message(self, client, userdata, message):
        self.received.set()

    def stop(self):
        self.client.disconnect()
        self.client.loop_stop()


@unittest.skipUnless(broker_is_available(), "Mosquitto must be running on localhost:1883")
class LocalMQTTIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        StatusWidget.SpeedWidget._widget = None
        StatusWidget.MapWidget._widget = None
        StatusWidget.BatteryWidget._widget = None
        self.window = None
        self.firmware = None
        self.probe = None

    def tearDown(self):
        if self.window is not None:
            self.window.close()
            self.app.processEvents()
        if self.firmware is not None:
            self.firmware.terminate()
            try:
                self.firmware.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.firmware.kill()
                self.firmware.wait(timeout=5)
        if self.probe is not None:
            self.probe.stop()

    def start_firmware(self):
        environment = os.environ.copy()
        environment["ROV_MQTT_FAKE_CLIENT_ID"] = f"fake_{uuid.uuid4().hex[:10]}"
        self.firmware = subprocess.Popen(
            [sys.executable, str(ROOT / "src" / "rov_mqtt" / "fake_firmware.py")],
            cwd=ROOT,
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        self.probe = HeartbeatProbe()
        self.assertTrue(self.probe.received.wait(5), "Fake Firmware heartbeat not received")

    def start_window(self):
        with patch.object(window_module, "Camera", CameraStub), patch.object(
            window_module,
            "PS4Controller",
            side_effect=RuntimeError("no physical controller needed for test"),
        ), patch.dict(os.environ, {"ROV_MQTT_BROKER_HOST": "localhost"}):
            self.window = Window()
        self.app.processEvents()

    def wait_for(self, predicate, description, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.app.processEvents()
            if predicate():
                return
            time.sleep(0.01)
        self.fail(f"Timed out waiting for {description}")

    def assert_live_gui_state(self):
        self.wait_for(
            lambda: self.window._telemetry_labels["Broker Connection:"].text() == "CONNECTED",
            "GUI broker connection",
        )
        self.wait_for(
            lambda: self.window._telemetry_labels["Robot Status:"].text() == "ONLINE",
            "plain-text robot online status",
        )
        self.wait_for(
            lambda: self.window._telemetry_labels["Operating Mode:"].text() == "ESTOP",
            "initial retained ESTOP mode",
        )
        self.wait_for(
            lambda: self.window._telemetry_labels["Battery Voltage:"].text() == "7.4 V",
            "canonical robot/telemetry subscription",
        )
        self.wait_for(
            lambda: "RTT:" in self.window._telemetry_labels["Comm Latency:"].text()
            and "ping_id: 0" not in self.window._telemetry_labels["Comm Latency:"].text(),
            "Ping/Pong response",
        )
        self.assertTrue(self.window._reset_estop_button.isEnabled())

    def test_gui_started_before_fake_firmware_and_controller_safety(self):
        self.start_window()
        self.wait_for(
            lambda: self.window._telemetry_labels["Broker Connection:"].text() == "CONNECTED",
            "GUI connection before simulator starts",
        )
        self.assertEqual(self.window._telemetry_labels["Robot Status:"].text(), "OFFLINE")
        self.assertEqual(self.window._telemetry_labels["Operating Mode:"].text(), "ESTOP")

        self.start_firmware()
        self.assert_live_gui_state()

        controller = ControllerStub()
        self.window.controller = controller
        self.window._poll_controller()
        self.assertEqual(self.window._telemetry_labels["Operating Mode:"].text(), "ESTOP")

        # Use the same button index as the PS4 Triangle mapping to exercise the
        # GUI edge detector, existing reset handler, MQTT client and simulator.
        controller.buttons = (False, False, False, True, False, False, False)
        self.window._poll_controller()
        self.window._poll_controller()
        self.assertEqual(self.window._operating_mode, "ESTOP")
        self.assertTrue(self.window._estop_latched)
        self.assertTrue(self.window._estop_reset_requested)
        self.wait_for(
            lambda: self.window._telemetry_labels["Operating Mode:"].text() == "MANUAL",
            "Triangle reset confirmed by Fake Firmware",
        )
        self.assertFalse(self.window._estop_latched)
        controller.buttons = (False,) * 7
        self.window._poll_controller()
        self.assertIsNone(self.firmware.poll(), "Fake Firmware exited after reset")

        controller.buttons = (True, False, False, False, False, False, False)
        self.window._poll_controller()
        self.wait_for(
            lambda: self.window._telemetry_labels["Arm Status:"].text() == "PICK",
            "PS4 PICK command reflected by fake telemetry",
        )
        self.wait_for(
            lambda: self.window._telemetry_labels["Robot Velocity:"].text() == "v: 0.40, w: -0.20",
            "PS4 drive command reflected by fake telemetry",
        )

        controller.buttons = (True, False, False, False, False, False, True)
        self.window._poll_controller()
        self.window._poll_controller()
        self.wait_for(
            lambda: self.window._telemetry_labels["Operating Mode:"].text() == "ESTOP",
            "OPTIONS ESTOP transition",
        )
        self.assertIsNone(self.firmware.poll(), "Fake Firmware exited after ESTOP")

    def test_fake_firmware_started_before_gui(self):
        self.start_firmware()
        self.start_window()
        self.assert_live_gui_state()


if __name__ == "__main__":
    unittest.main()

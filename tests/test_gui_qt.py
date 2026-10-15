import os
import json
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "gui"))

import numpy as np
from PySide6.QtWidgets import QApplication

import StatusWidget
import window as window_module
from data import SpeedData
from window import Window


class FakeMQTTClient:
    def __init__(self, **kwargs):
        self.message_callback = None
        self.connection_callback = None
        self.commands = []

    def set_message_callback(self, callback):
        self.message_callback = callback

    def set_connection_callback(self, callback):
        self.connection_callback = callback

    def connect(self):
        self.connection_callback(True)

    def disconnect(self):
        pass

    def publish_drive(self, *args): self.commands.append(("drive", args))
    def publish_arm(self, *args): self.commands.append(("arm", args))
    def publish_estop(self, *args):
        self.commands.append(("estop", args))
        return SimpleNamespace(rc=0, mid=len(self.commands))
    def publish_estop_reset(self):
        self.commands.append(("estop_reset", ()))
        return SimpleNamespace(rc=0, mid=len(self.commands))
    def publish_ping(self, **kwargs): self.commands.append(("ping", kwargs))


class CameraStub:
    def __init__(self):
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)

    def release(self):
        pass


class ControllerStub:
    def __init__(self, buttons=()):
        self.buttons = tuple(buttons)
        self.drive = (0.4, -0.2)

    def read_drive(self):
        return self.drive

    def read_buttons(self):
        return self.buttons

    def close(self):
        pass


def mode_message(mode):
    return json.dumps({
        "mode": mode,
        "ver": 1,
        "sid": "fake-firmware",
        "seq": 1,
    })


class QtIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        StatusWidget.SpeedWidget._widget = None
        StatusWidget.MapWidget._widget = None
        StatusWidget.BatteryWidget._widget = None
        self.client = FakeMQTTClient()
        with patch.object(window_module, "Camera", CameraStub), patch.object(
            window_module, "MQTTClient", return_value=self.client
        ), patch.object(
            window_module,
            "PS4Controller",
            side_effect=RuntimeError("no test controller"),
        ):
            self.window = Window()
        self.app.processEvents()

    def tearDown(self):
        self.window.close()
        self.app.processEvents()

    def wait_for(self, predicate, description, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.app.processEvents()
            if predicate():
                return
            time.sleep(0.01)
        self.fail(f"Timed out waiting for {description}")

    def test_qml_speed_binding_and_queued_mqtt_telemetry(self):
        self.assertEqual(self.window.speed_widget.status(), self.window.speed_widget.Status.Ready)
        self.assertTrue(self.window._estop_button.isEnabled())

        telemetry = (
            "robot/telemetry",
            '{"v":0.6,"w":0.2,"x":1.5,"y":2.0,"theta":0.1,'
            '"arm":"STOW","batt_v":7.4,"pose_valid":true,"link_ok":true}',
        )
        worker = threading.Thread(target=self.client.message_callback, args=telemetry)
        worker.start()
        worker.join(timeout=2)
        self.wait_for(
            lambda: self.window._telemetry_labels["Battery Voltage:"].text() == "7.4 V",
            "Qt queued telemetry update",
        )
        self.assertAlmostEqual(self.window.speed_widget.rootObject().property("speed"), 0.6)
        self.assertEqual(self.window._telemetry_labels["Robot Position:"].text(), "x: 1.50, y: 2.00")

    def test_initial_window_fits_available_screen_and_keeps_safety_controls_visible(self):
        screen = self.window.screen()
        available = screen.availableGeometry()
        frame = self.window.frameGeometry()

        self.assertLessEqual(frame.width(), available.width())
        self.assertLessEqual(frame.height(), available.height())
        self.assertGreaterEqual(frame.left(), available.left())
        self.assertGreaterEqual(frame.top(), available.top())
        self.assertLessEqual(frame.right(), available.right())
        self.assertLessEqual(frame.bottom(), available.bottom())
        self.assertTrue(self.window._estop_button.isVisible())
        self.assertTrue(self.window._reset_estop_button.isVisible())

    def test_estop_reset_requires_state_confirmation_before_commands(self):
        self.client.connection_callback(True)
        self.client.message_callback("robot/status/online", "online")
        self.client.message_callback("robot/state/mode", mode_message("ESTOP"))
        self.app.processEvents()
        self.assertTrue(self.window._estop_button.isEnabled())
        self.assertTrue(self.window._reset_estop_button.isEnabled())

        self.window._reset_estop_button.click()
        self.assertIn(("estop_reset", ()), self.client.commands)
        self.assertFalse(self.window._reset_estop_button.isEnabled())
        self.assertTrue(self.window._estop_reset_requested)
        self.assertTrue(self.window._estop_latched)

        # A duplicate/retained ESTOP must not erase the pending request.
        self.client.message_callback("robot/state/mode", mode_message("ESTOP"))
        self.app.processEvents()
        self.assertTrue(self.window._estop_reset_requested)
        self.assertTrue(self.window._estop_latched)

        # An un-enveloped MANUAL payload is not an authoritative confirmation.
        self.client.message_callback("robot/state/mode", '{"mode":"MANUAL"}')
        self.assertEqual(self.window._operating_mode, "ESTOP")
        self.assertTrue(self.window._estop_reset_requested)
        self.assertTrue(self.window._estop_latched)

        self.client.message_callback("robot/state/mode", mode_message("MANUAL"))
        self.app.processEvents()
        self.assertEqual(self.window._telemetry_labels["Operating Mode:"].text(), "MANUAL")
        self.assertFalse(self.window._reset_estop_button.isEnabled())

        self.window._estop_button.click()
        self.assertIn(("estop", ()), self.client.commands)
        self.client.message_callback("robot/state/mode", mode_message("ESTOP"))
        self.app.processEvents()
        self.assertTrue(self.window._reset_estop_button.isEnabled())

    def test_controller_commands_require_online_confirmed_manual_and_latch_clear(self):
        controller = ControllerStub([True, True, True, False, False, False, False])
        self.window.controller = controller

        # Startup is offline and latched, so neither drive nor arm commands pass.
        self.window._poll_controller()
        self.assertFalse(any(command[0] in {"drive", "arm"} for command in self.client.commands))

        self.client.message_callback("robot/status/online", "online")
        self.client.message_callback("robot/state/mode", mode_message("ESTOP"))
        self.window._poll_controller()
        self.assertFalse(any(command[0] in {"drive", "arm"} for command in self.client.commands))

        controller.buttons = (False,) * 7
        self.window._poll_controller()
        self.window._send_estop_reset()
        self.assertTrue(self.window._estop_reset_requested)

        # The pending publish alone cannot enable drive or arm commands, even
        # while the operator is holding those buttons.
        controller.buttons = (True, True, True, False, False, False, False)
        self.window._poll_controller()
        self.assertEqual(self.window._operating_mode, "ESTOP")
        self.assertTrue(self.window._estop_latched)
        self.assertFalse(any(name in {"drive", "arm"} for name, _ in self.client.commands))

        controller.buttons = (False,) * 7
        self.window._poll_controller()
        self.client.message_callback("robot/state/mode", '{"mode":"MANUAL"}')
        self.assertTrue(self.window._estop_latched)
        self.client.message_callback("robot/state/mode", mode_message("MANUAL"))
        self.assertTrue(self.window._broker_connected)
        self.assertTrue(self.window._robot_online)
        self.assertFalse(self.window._estop_latched)
        controller.buttons = (True, True, True, False, False, False, False)
        self.window._poll_controller()
        arm_commands = [args[0] for name, args in self.client.commands if name == "arm"]
        self.assertCountEqual(arm_commands, ["PICK", "DROP", "STOW"])
        self.assertIn(("drive", controller.drive), self.client.commands)

        # Held buttons don't repeat actions; only drive is periodically refreshed.
        self.window._poll_controller()
        self.assertEqual(
            [args[0] for name, args in self.client.commands if name == "arm"],
            ["PICK", "DROP", "STOW"],
        )

        # OPTIONS is a rising-edge ESTOP request and latches drive immediately.
        controller.buttons = (True, True, True, False, False, False, True)
        self.window._poll_controller()
        self.window._poll_controller()
        self.assertEqual(sum(name == "estop" for name, _ in self.client.commands), 1)
        drive_count = sum(name == "drive" for name, _ in self.client.commands)
        self.window._poll_controller()
        self.assertEqual(sum(name == "drive" for name, _ in self.client.commands), drive_count)

    def test_options_button_press_edges_request_estop_outside_manual_mode(self):
        controller = ControllerStub([False] * 6 + [True])
        self.window.controller = controller
        self.client.connection_callback(True)
        self.window._robot_online = False
        self.window._operating_mode = "ESTOP"
        self.window._estop_reset_requested = True

        # The safety input is independent of online/MANUAL and drive gating.
        self.window._poll_controller()
        self.assertEqual(
            sum(name == "estop" for name, _ in self.client.commands), 1
        )
        self.assertTrue(self.window._estop_latched)
        self.assertFalse(self.window._estop_reset_requested)

        # Holding OPTIONS is one press; release and press again is a new one.
        self.window._poll_controller()
        self.assertEqual(
            sum(name == "estop" for name, _ in self.client.commands), 1
        )
        controller.buttons = (False,) * 7
        self.window._poll_controller()
        controller.buttons = (False,) * 6 + (True,)
        self.window._poll_controller()
        self.assertEqual(
            sum(name == "estop" for name, _ in self.client.commands), 2
        )
        self.assertFalse(
            any(name in {"drive", "arm"} for name, _ in self.client.commands)
        )

    def test_triangle_button_uses_reset_handler_and_waits_for_manual_confirmation(self):
        controller = ControllerStub((False, False, False, True, False, False, False))
        self.window.controller = controller
        self.client.connection_callback(True)
        self.client.message_callback("robot/status/online", "online")
        self.client.message_callback("robot/state/mode", mode_message("ESTOP"))

        self.window._poll_controller()
        reset_count = sum(name == "estop_reset" for name, _ in self.client.commands)
        self.assertEqual(reset_count, 1)
        self.assertTrue(self.window._estop_reset_requested)
        self.assertTrue(self.window._estop_latched)

        # Holding Triangle does not repeat the reset, and ESTOP stays latched.
        self.window._poll_controller()
        self.assertEqual(
            sum(name == "estop_reset" for name, _ in self.client.commands), 1
        )
        self.assertFalse(
            any(name in {"drive", "arm"} for name, _ in self.client.commands)
        )
        self.client.message_callback("robot/state/mode", mode_message("ESTOP"))
        self.assertTrue(self.window._estop_reset_requested)
        self.assertTrue(self.window._estop_latched)

        # Only the enveloped MANUAL confirmation clears the latch.
        self.client.message_callback("robot/state/mode", mode_message("MANUAL"))
        self.assertFalse(self.window._estop_reset_requested)
        self.assertFalse(self.window._estop_latched)
        self.window._poll_controller()
        self.assertTrue(any(name == "drive" for name, _ in self.client.commands))

        # Release then press makes a fresh edge; the handler still checks mode.
        controller.buttons = (False,) * 7
        self.window._poll_controller()
        self.client.message_callback("robot/state/mode", mode_message("ESTOP"))
        controller.buttons = (False, False, False, True, False, False, False)
        self.window._poll_controller()
        self.assertEqual(
            sum(name == "estop_reset" for name, _ in self.client.commands), 2
        )

    def test_mqtt_disconnect_and_controller_loss_latch_drive_until_reset_confirmation(self):
        controller = ControllerStub([False] * 7)
        self.window.controller = controller
        self.window._broker_connected = True
        self.window.update_robot_status(True)
        self.window.update_operating_mode("MANUAL")

        self.window._update_broker_connection(False)
        self.window._update_broker_connection(True)
        self.window.update_robot_status(True)
        self.window.update_operating_mode("MANUAL")
        self.window._poll_controller()
        self.assertTrue(self.window._estop_latched)
        self.assertFalse(any(name == "drive" for name, _ in self.client.commands))

        self.window._send_estop_reset()
        self.window.update_operating_mode("MANUAL")
        controller.read_drive = lambda: (_ for _ in ()).throw(RuntimeError("disconnected"))
        self.window._poll_controller()
        self.assertTrue(self.window._estop_latched)
        controller.read_drive = lambda: (0.4, -0.2)
        self.window._poll_controller()
        self.assertFalse(any(name == "drive" for name, _ in self.client.commands))

    def test_reset_publish_rejection_keeps_estop_latched(self):
        self.client.connection_callback(True)
        self.client.message_callback("robot/status/online", "online")
        self.client.message_callback("robot/state/mode", mode_message("ESTOP"))
        self.client.publish_estop_reset = lambda: SimpleNamespace(rc=15, mid=9)

        self.window._send_estop_reset()

        self.assertFalse(self.window._estop_reset_requested)
        self.assertTrue(self.window._estop_latched)
        self.assertFalse(any(name in {"drive", "arm"} for name, _ in self.client.commands))

    def test_speed_data_clamps_for_qml_gauge(self):
        bridge = SpeedData()
        bridge.speed = 1.5
        self.assertEqual(bridge.speed, 1.0)
        bridge.speed = -0.5
        self.assertEqual(bridge.speed, 0.0)


if __name__ == "__main__":
    unittest.main()

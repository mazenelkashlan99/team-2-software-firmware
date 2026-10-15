import json
import os
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "gui"))

from rov_mqtt.mqtt_client import MQTTClient
from window import Window


class MQTTClientTests(unittest.TestCase):
    def setUp(self):
        self.mqtt = MQTTClient(client_id="unit_test_client")
        self.published = []
        self.mqtt.client.publish = lambda topic, payload, qos=0: self.published.append(
            (topic, json.loads(payload), qos)
        )

    def test_shared_envelope_sequence_including_heartbeat(self):
        heartbeat_sent = threading.Event()

        def publish(topic, payload, qos=0):
            self.published.append((topic, json.loads(payload), qos))
            if topic == "robot/hb/console":
                heartbeat_sent.set()

        self.mqtt.client.publish = publish
        self.mqtt.start_console_heartbeat(lambda: {"ts": 123}, rate_hz=20)
        self.assertTrue(heartbeat_sent.wait(2))
        self.mqtt.stop_console_heartbeat()
        self.mqtt.publish_drive(0.25, -0.5)

        sequences = [entry[1]["seq"] for entry in self.published]
        self.assertEqual(sequences, list(range(1, len(sequences) + 1)))
        self.assertTrue(all(entry[1]["ver"] == 1 for entry in self.published))
        self.assertTrue(all(entry[1]["sid"] == self.mqtt.sid for entry in self.published))
        self.assertGreaterEqual(len(self.mqtt.sid), 1)
        self.assertLessEqual(len(self.mqtt.sid), 15)
        self.assertIn("robot/hb/console", [entry[0] for entry in self.published])

    def test_broker_host_can_be_selected_from_environment_or_constructor(self):
        with patch.dict(os.environ, {"ROV_MQTT_BROKER_HOST": "172.20.10.3"}):
            configured = MQTTClient()
            explicit = MQTTClient(broker_host="broker.example")

        self.assertEqual(configured.broker_host, "172.20.10.3")
        self.assertEqual(configured.broker_port, 1883)
        self.assertEqual(explicit.broker_host, "broker.example")

    def test_connect_uses_and_reports_the_configured_broker_endpoint(self):
        with patch.dict(os.environ, {"ROV_MQTT_BROKER_HOST": "172.20.10.3"}):
            client = MQTTClient()
        client.client.connect_async = Mock()
        client.client.loop_start = Mock()

        with patch("builtins.print") as log:
            client.connect()

        client.client.connect_async.assert_called_once_with("172.20.10.3", 1883)
        self.assertIn("172.20.10.3:1883", log.call_args.args[0])

    def test_supported_command_payloads_and_validation(self):
        self.mqtt.publish_drive(1, -1)
        for action in ("PICK", "DROP", "STOW"):
            self.mqtt.publish_arm(action)
        self.mqtt.publish_estop()
        self.mqtt.publish_estop_reset()
        self.mqtt.publish_ping(42)

        by_topic = {topic: payload for topic, payload, _ in self.published}
        qos_by_topic = {topic: qos for topic, _, qos in self.published}
        self.assertEqual(by_topic["robot/cmd/drive"]["thr"], 1)
        self.assertEqual(
            [payload["action"] for topic, payload, _ in self.published if topic == "robot/cmd/arm"],
            ["PICK", "DROP", "STOW"],
        )
        self.assertEqual(
            by_topic["robot/cmd/estop"],
            {"ver": 1, "sid": self.mqtt.sid, "seq": 5},
        )
        self.assertEqual(
            by_topic["robot/cmd/estop_reset"],
            {"ver": 1, "sid": self.mqtt.sid, "seq": 6},
        )
        self.assertEqual(by_topic["robot/cmd/ping"]["ping_id"], 42)
        self.assertNotIn("t_send", by_topic["robot/cmd/ping"])
        self.assertEqual(qos_by_topic["robot/cmd/drive"], 0)
        self.assertEqual(qos_by_topic["robot/cmd/arm"], 1)
        self.assertEqual(qos_by_topic["robot/cmd/estop"], 1)
        self.assertEqual(qos_by_topic["robot/cmd/estop_reset"], 1)
        self.assertEqual(qos_by_topic["robot/cmd/ping"], 0)

        for action in (lambda: self.mqtt.publish_drive(True, 0),
                       lambda: self.mqtt.publish_drive(float("nan"), 0),
                       lambda: self.mqtt.publish_drive(1.1, 0),
                       lambda: self.mqtt.publish_arm("FLY"),
                       lambda: self.mqtt.publish_ping(True)):
            with self.assertRaises(ValueError):
                action()

        self.assertFalse(hasattr(self.mqtt, "publish_start"))
        self.assertFalse(hasattr(self.mqtt, "publish_mission"))
        self.assertFalse(hasattr(self.mqtt, "publish_mode"))
        self.assertFalse(hasattr(self.mqtt, "publish_pose_set"))
        self.assertFalse(hasattr(self.mqtt, "publish_qr"))
        self.assertFalse(hasattr(self.mqtt, "publish_color"))

    def test_estop_reset_publish_reports_queue_result_without_claiming_confirmation(self):
        broker_result = SimpleNamespace(rc=0, mid=17)
        self.mqtt.client.publish = Mock(return_value=broker_result)

        with patch("builtins.print") as log:
            result = self.mqtt.publish_estop_reset()

        self.assertIs(result, broker_result)
        topic, payload = self.mqtt.client.publish.call_args.args
        qos = self.mqtt.client.publish.call_args.kwargs["qos"]
        self.assertEqual(topic, "robot/cmd/estop_reset")
        self.assertEqual(json.loads(payload), {
            "ver": 1,
            "sid": self.mqtt.sid,
            "seq": 1,
        })
        self.assertEqual(qos, 1)
        self.assertIn("awaiting MANUAL confirmation", log.call_args.args[0])

    def test_subscription_list_contains_only_supported_incoming_topics(self):
        self.mqtt._heartbeat_envelope_factory = None
        self.mqtt.client.subscribe = Mock()
        connection_changes = []
        self.mqtt.set_connection_callback(connection_changes.append)
        self.mqtt._on_connect(self.mqtt.client, None, None, 0, None)
        topics = [call.args[0] for call in self.mqtt.client.subscribe.call_args_list]

        self.assertEqual(
            topics,
            [
                "robot/telemetry",
                "robot/state/mode",
                "robot/status/online",
                "robot/hb/robot",
                "robot/state/pong",
                "robot/log",
            ],
        )
        self.mqtt._on_disconnect(self.mqtt.client, None, None, "network lost", None)
        self.assertEqual(connection_changes, [True, False])


class MessageHandlerTests(unittest.TestCase):
    class Handler:
        _has_valid_envelope = staticmethod(Window._has_valid_envelope)
        _number_or_default = staticmethod(Window._number_or_default)

        def __init__(self):
            self._pending_pings = {}
            self.status = None
            self.latencies = []
            self.logs = []
            self.mode = None
            self.telemetry = []
            self.link_status = []
            self.battery_voltages = []
            self.battery = Mock()
            self._telemetry_labels = {"Battery Voltage:": Mock()}

        def update_robot_status(self, value): self.status = value
        def update_communication_latency(self, ping_id, rtt): self.latencies.append((ping_id, rtt))
        def update_system_logs(self, value): self.logs.append(value)
        def update_operating_mode(self, value): self.mode = value
        def speed_changed(self, *values): pass
        def position_changed(self, *values): pass
        def update_arm_status(self, value): pass
        def update_position_validity(self, value): pass
        def update_communication_link(self, value): self.link_status.append(value)
        def battery_changed(self, value): self.battery_voltages.append(value)

    def test_plain_text_online_and_offline(self):
        handler = self.Handler()
        Window.handle_mqtt_message(handler, "robot/status/online", "online")
        self.assertIs(handler.status, True)
        Window.handle_mqtt_message(handler, "robot/status/online", "offline")
        self.assertIs(handler.status, False)

    def test_online_status_is_plain_text_and_robot_heartbeat_is_not_online_confirmation(self):
        handler = self.Handler()
        Window.handle_mqtt_message(
            handler, "robot/status/online", '{"status":"online"}'
        )
        self.assertIsNone(handler.status)
        Window.handle_mqtt_message(
            handler, "robot/hb/robot", '{"ver":1,"sid":"robot-sim","seq":1}'
        )
        self.assertIsNone(handler.status)

    def test_mode_requires_envelope_and_sid_robot_sim_is_valid(self):
        handler = self.Handler()
        Window.handle_mqtt_message(
            handler,
            "robot/state/mode",
            '{"mode":"MANUAL","ver":1,"sid":"robot-sim","seq":1}',
        )
        self.assertEqual(handler.mode, "MANUAL")
        Window.handle_mqtt_message(handler, "robot/state/mode", '{"mode":"ESTOP"}')
        self.assertEqual(handler.mode, "MANUAL")

    def test_only_canonical_robot_telemetry_updates_comm_link(self):
        handler = self.Handler()
        payload = '{"link_ok":true,"pose_valid":true,"v":0,"w":0}'
        Window.handle_mqtt_message(handler, "robot/state/telemetry", payload)
        self.assertEqual(handler.link_status, [])
        Window.handle_mqtt_message(handler, "robot/telemetry", payload)
        self.assertEqual(handler.link_status, [True])

    def test_pong_uses_local_timestamp_and_ignores_unknown_or_duplicate_ids(self):
        handler = self.Handler()
        handler._pending_pings[7] = 100.0
        with unittest.mock.patch("window.time.monotonic", return_value=100.025):
            Window.handle_mqtt_message(handler, "robot/state/pong", '{"ping_id":7}')
            Window.handle_mqtt_message(handler, "robot/state/pong", '{"ping_id":7}')
            Window.handle_mqtt_message(handler, "robot/state/pong", '{"ping_id":99}')
        self.assertEqual(handler.latencies, [(7, 25.0)])
        self.assertNotIn(7, handler._pending_pings)

    def test_logs_read_firmware_lvl_and_msg_fields(self):
        handler = self.Handler()
        Window.handle_mqtt_message(handler, "robot/log", '{"lvl":"WARN","msg":"test"}')
        self.assertEqual(handler.logs, ["WARN: test"])

    def test_bad_json_and_wrong_payload_shapes_are_ignored(self):
        handler = self.Handler()
        Window.handle_mqtt_message(handler, "robot/log", "not-json")
        Window.handle_mqtt_message(handler, "robot/state/mode", "[]")
        self.assertEqual(handler.logs, [])
        self.assertIsNone(handler.mode)


if __name__ == "__main__":
    unittest.main()

import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rov_mqtt import fake_firmware


class FakeMQTTClient:
    def __init__(self):
        self.subscriptions = []
        self.published = []

    def subscribe(self, subscriptions):
        self.subscriptions.extend(subscriptions)

    def publish(self, topic, payload, qos=0, retain=False):
        self.published.append((topic, payload, qos, retain))
        return SimpleNamespace(
            rc=0,
            mid=len(self.published),
            wait_for_publish=lambda timeout=None: True,
        )


class FakeFirmwareTests(unittest.TestCase):
    def setUp(self):
        fake_firmware.connected.clear()
        fake_firmware.SID = "fake-firmware-1"
        fake_firmware.outgoing_seq = 0
        fake_firmware.robot_mode = "ESTOP"
        fake_firmware.drive_state.update(thr=0.0, turn=0.0)
        fake_firmware.arm_state = "STOW"
        self.client = FakeMQTTClient()

    def send_command(self, topic, payload):
        message = SimpleNamespace(
            topic=topic,
            payload=json.dumps(payload).encode("utf-8"),
        )
        fake_firmware.on_message(self.client, None, message)

    def test_connect_publishes_plain_online_and_retained_estop(self):
        fake_firmware.on_connect(self.client, None, None, 0, None)

        online = next(item for item in self.client.published if item[0] == "robot/status/online")
        mode = next(item for item in self.client.published if item[0] == "robot/state/mode")
        self.assertEqual(online[1:3], ("online", 1))
        self.assertTrue(online[3])
        self.assertEqual(json.loads(mode[1])["mode"], "ESTOP")
        self.assertEqual(mode[2:], (1, True))
        self.assertTrue(fake_firmware.connected.is_set())
        self.assertIn(("robot/cmd/estop_reset", 1), self.client.subscriptions)
        self.assertIn(("robot/cmd/ping", 0), self.client.subscriptions)

    def test_ping_estop_reset_and_commands_follow_supported_protocol(self):
        fake_firmware.on_connect(self.client, None, None, 0, None)

        # Commands emitted by MQTTClient carry the protocol envelope.
        base = {"ver": 1, "sid": "console", "seq": 1}
        self.send_command("robot/cmd/ping", {**base, "ping_id": 24})
        self.send_command("robot/cmd/drive", {**base, "thr": 0.5, "turn": -0.2})
        self.assertEqual(fake_firmware.drive_state, {"thr": 0.0, "turn": 0.0})

        self.send_command("robot/cmd/estop_reset", base)
        self.assertEqual(fake_firmware.robot_mode, "MANUAL")
        mode_messages = [
            (json.loads(payload)["mode"], qos, retain)
            for topic, payload, qos, retain in self.client.published
            if topic == "robot/state/mode"
        ]
        self.assertEqual(
            mode_messages,
            [("ESTOP", 1, True), ("MANUAL", 1, True)],
        )

        self.send_command("robot/cmd/drive", {**base, "thr": 0.5, "turn": -0.2})
        self.send_command("robot/cmd/arm", {**base, "action": "PICK"})
        self.assertEqual(fake_firmware.drive_state, {"thr": 0.5, "turn": -0.2})
        self.assertEqual(fake_firmware.arm_state, "PICK")

        self.send_command("robot/cmd/estop", base)
        self.assertEqual(fake_firmware.robot_mode, "ESTOP")
        self.assertEqual(mode_messages[-1][0], "MANUAL")
        confirmed_modes = [
            json.loads(payload)["mode"]
            for topic, payload, _, _ in self.client.published
            if topic == "robot/state/mode"
        ]
        self.assertEqual(confirmed_modes, ["ESTOP", "MANUAL", "ESTOP"])
        self.send_command("robot/cmd/drive", {**base, "thr": 0.7, "turn": 0.1})
        self.send_command("robot/cmd/arm", {**base, "action": "DROP"})
        self.assertEqual(fake_firmware.robot_mode, "ESTOP")
        self.assertEqual(fake_firmware.drive_state, {"thr": 0.0, "turn": 0.0})
        self.assertEqual(fake_firmware.arm_state, "PICK")

        pong = next(
            json.loads(payload)
            for topic, payload, _, _ in self.client.published
            if topic == "robot/state/pong"
        )
        self.assertEqual(pong["ping_id"], 24)
        self.assertNotIn("t_send", pong)
        self.assertEqual(pong["ver"], 1)
        self.assertEqual(pong["sid"], fake_firmware.SID)
        self.assertEqual(
            [item["seq"] for topic, payload, _, _ in self.client.published
             if topic != "robot/status/online"
             for item in [json.loads(payload)]],
            list(range(1, 5)),
        )

    def test_fake_firmware_rejects_invalid_reset_without_publishing_manual(self):
        fake_firmware.on_connect(self.client, None, None, 0, None)
        self.send_command(
            "robot/cmd/estop_reset",
            {"ver": 1, "sid": "console", "seq": 1, "mode": "MANUAL"},
        )

        modes = [
            json.loads(payload)["mode"]
            for topic, payload, _, _ in self.client.published
            if topic == "robot/state/mode"
        ]
        self.assertEqual(fake_firmware.robot_mode, "ESTOP")
        self.assertEqual(modes, ["ESTOP"])


if __name__ == "__main__":
    unittest.main()

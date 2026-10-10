import time

from src.rov_mqtt.mqtt_client import MQTTClient


def on_message(topic: str, payload: str) -> None:
    print(f"[RECEIVED] {topic} -> {payload}")


def run_test(name, action, should_reject=False):
    try:
        action()

    except ValueError as error:
        if should_reject:
            print(f"[PASS] {name}: rejected correctly ({error})")
        else:
            print(f"[FAIL] {name}: unexpected rejection ({error})")

    except Exception as error:
        print(f"[FAIL] {name}: {type(error).__name__}: {error}")

    else:
        if should_reject:
            print(f"[FAIL] {name}: invalid input was accepted")
        else:
            print(f"[PASS] {name}: arguments accepted; verify delivery in Mock")


def make_box(box_id):
    return {
        "id": box_id,
        "type": "TEST",
        "x": 0.5,
        "y": 0.5,
    }


def make_zone():
    return {
        "type": "TEST",
        "x": 2.0,
        "y": 2.0,
    }


def main():
    mqtt_client = MQTTClient()
    mqtt_client.set_message_callback(on_message)

    try:
        print("Connecting to MQTT broker...")
        mqtt_client.connect()
        time.sleep(1)

        print("\n========== DRIVE TESTS ==========")

        # Valid drive values
        run_test(
            "Drive valid values",
            lambda: mqtt_client.publish_drive(0.5, -0.2),
        )

        # Boundary values should be accepted
        run_test(
            "Drive lower boundary",
            lambda: mqtt_client.publish_drive(-1.0, -1.0),
        )

        run_test(
            "Drive upper boundary",
            lambda: mqtt_client.publish_drive(1.0, 1.0),
        )

        # Invalid throttle
        run_test(
            "Drive throttle too high",
            lambda: mqtt_client.publish_drive(1.5, 0.0),
            should_reject=True,
        )

        run_test(
            "Drive throttle too low",
            lambda: mqtt_client.publish_drive(-1.5, 0.0),
            should_reject=True,
        )

        # Invalid turn
        run_test(
            "Drive turn too high",
            lambda: mqtt_client.publish_drive(0.0, 1.5),
            should_reject=True,
        )

        run_test(
            "Drive turn too low",
            lambda: mqtt_client.publish_drive(0.0, -1.5),
            should_reject=True,
        )

        print("\n========== MODE TESTS ==========")

        for mode in ("MANUAL", "SEMI", "AUTO"):
            run_test(
                f"Mode {mode}",
                lambda mode=mode: mqtt_client.publish_mode(mode),
            )

        run_test(
            "Invalid mode",
            lambda: mqtt_client.publish_mode("INVALID"),
            should_reject=True,
        )

        print("\n========== ARM TESTS ==========")

        for action in ("PICK", "DROP", "STOW"):
            run_test(
                f"Arm {action}",
                lambda action=action: mqtt_client.publish_arm(action),
            )

        run_test(
            "Invalid arm action",
            lambda: mqtt_client.publish_arm("FLY"),
            should_reject=True,
        )

        print("\n========== MISSION TESTS ==========")

        valid_mission = {
            "boxes": [make_box(1)],
            "zones": [make_zone()],
        }

        run_test(
            "Valid mission",
            lambda: mqtt_client.publish_mission(valid_mission),
        )

        # Missing boxes list
        run_test(
            "Mission missing boxes",
            lambda: mqtt_client.publish_mission({
                "zones": [make_zone()],
            }),
            should_reject=True,
        )

        # Missing zones list
        run_test(
            "Mission missing zones",
            lambda: mqtt_client.publish_mission({
                "boxes": [make_box(1)],
            }),
            should_reject=True,
        )

        # More than 8 boxes
        too_many_boxes = {
            "boxes": [make_box(i) for i in range(1, 10)],
            "zones": [make_zone()],
        }

        run_test(
            "Mission with 9 boxes",
            lambda: mqtt_client.publish_mission(too_many_boxes),
            should_reject=True,
        )

        # More than 4 zones
        too_many_zones = {
            "boxes": [make_box(1)],
            "zones": [make_zone() for _ in range(5)],
        }

        run_test(
            "Mission with 5 zones",
            lambda: mqtt_client.publish_mission(too_many_zones),
            should_reject=True,
        )

        # Missing a required box field
        invalid_box = {
            "boxes": [{"id": 1, "type": "TEST", "x": 0.5}],
            "zones": [make_zone()],
        }

        run_test(
            "Mission box missing y",
            lambda: mqtt_client.publish_mission(invalid_box),
            should_reject=True,
        )

        print("\n========== OTHER PUBLISH TESTS ==========")

        run_test(
            "Pose set",
            lambda: mqtt_client.publish_pose_set(
                x=0.5,
                y=1.2,
                theta=0.3,
            ),
        )

        run_test(
            "Ping request",
            lambda: mqtt_client.publish_ping(
                ping_id=100,
                t_send=time.time(),
            ),
        )

        run_test(
            "QR publication",
            lambda: mqtt_client.publish_qr(
                valid=True,
                raw="TEST_QR",
                x=0.5,
                y=0.5,
            ),
        )

        # Transport smoke test only; this is not the agreed color schema.
        run_test(
            "Color publication smoke test",
            lambda: mqtt_client.publish_color({
                "test": True,
            }),
        )

        print("\n========== WAITING FOR MESSAGES ==========")
        print("Check fake_firmware.py terminal for published messages.")
        print("Waiting 3 seconds for MQTT messages and Pong...")

        time.sleep(3)

    except KeyboardInterrupt:
        print("\nTests interrupted by user.")

    finally:
        mqtt_client.disconnect()
        print("MQTT test client disconnected.")


if __name__ == "__main__":
    main()

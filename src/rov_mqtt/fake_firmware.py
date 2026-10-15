
import json
import os
import time
from threading import Event, Lock
from uuid import uuid4

import paho.mqtt.client as mqtt

BROKER_HOST = "localhost"
BROKER_PORT = 1883

connected = Event()

# One session ID for this Fake Firmware session
SID = uuid4().hex[:15]

# One shared sequence counter for all outgoing application messages
outgoing_seq = 0
sequence_lock = Lock()
state_lock = Lock()
robot_mode = "ESTOP"
drive_state = {"thr": 0.0, "turn": 0.0}
arm_state = "STOW"


def add_envelope(payload: dict) -> dict:
    global outgoing_seq

    with sequence_lock:
        outgoing_seq += 1
        seq = outgoing_seq

    message = dict(payload)
    message.update({
        "ver": 1,
        "sid": SID,
        "seq": seq,
    })

    return message


def publish_json(client, topic, payload, qos=0, retain=False):
    message = add_envelope(payload)

    return client.publish(
        topic,
        json.dumps(message),
        qos=qos,
        retain=retain,
    )


def has_valid_envelope(message: dict) -> bool:
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


def on_connect(client, userdata, flags, reason_code, properties):
    print(f"[FAKE FIRMWARE] Connected: {reason_code}")

    if reason_code != 0:
        print("[FAKE FIRMWARE] Connection failed")
        return

    client.subscribe([
        ("robot/cmd/drive", 0),
        ("robot/cmd/arm", 1),
        ("robot/cmd/estop", 1),
        ("robot/cmd/estop_reset", 1),
        ("robot/cmd/ping", 0),
        ("robot/hb/console", 0),
    ])

    client.publish("robot/status/online", "online", qos=1, retain=True)
    publish_json(
        client,
        "robot/state/mode",
        {"mode": "ESTOP"},
        qos=1,
        retain=True,
    )

    print("[FAKE FIRMWARE] Subscriptions requested.")
    connected.set()


def on_message(client, userdata, message):
    global robot_mode, arm_state
    payload = message.payload.decode("utf-8", errors="replace")

    print(f"\n[RECEIVED] {message.topic}")
    if message.topic == "robot/cmd/estop_reset":
        print("[FAKE FIRMWARE] ESTOP reset command received")

    try:
        data = json.loads(payload)
        print(json.dumps(data, indent=2))
    except json.JSONDecodeError:
        print(payload)
        return

    if not isinstance(data, dict):
        return
    if message.topic.startswith("robot/cmd/") and not has_valid_envelope(data):
        reason = "invalid version/session/sequence envelope"
        print(f"[FAKE FIRMWARE] Rejected {message.topic}: {reason}")
        return

    if message.topic == "robot/cmd/ping":
        ping_id = data.get("ping_id")
        if isinstance(ping_id, bool) or not isinstance(ping_id, int) or ping_id < 0:
            print("[FAKE FIRMWARE] Ignored invalid ping request")
            return
        pong = {
            "ping_id": ping_id,
        }

        publish_json(
            client,
            "robot/state/pong",
            pong,
            qos=0,
        )

        print("[SENT] robot/state/pong")
    elif message.topic == "robot/cmd/estop":
        with state_lock:
            robot_mode = "ESTOP"
            drive_state.update(thr=0.0, turn=0.0)
        print("[FAKE FIRMWARE] ESTOP command accepted; publishing ESTOP mode confirmation")
        result = publish_json(
            client,
            "robot/state/mode",
            {"mode": robot_mode},
            qos=1,
            retain=True,
        )
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"[FAKE FIRMWARE] ESTOP mode confirmation queued (mid={result.mid})")
        else:
            print(f"[FAKE FIRMWARE] ESTOP mode publish failed (rc={result.rc})")
    elif message.topic == "robot/cmd/estop_reset":
        if set(data) != {"ver", "sid", "seq"}:
            print("[FAKE FIRMWARE] Rejected ESTOP reset: expected an empty command payload")
            return
        with state_lock:
            if robot_mode != "ESTOP":
                print(f"[FAKE FIRMWARE] Rejected ESTOP reset: current mode is {robot_mode}")
                return
            robot_mode = "MANUAL"
        result = publish_json(
            client,
            "robot/state/mode",
            {"mode": robot_mode},
            qos=1,
            retain=True,
        )
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(
                f"[FAKE FIRMWARE] ESTOP reset accepted; MANUAL state confirmation "
                f"queued (mid={result.mid})"
            )
        else:
            print(f"[FAKE FIRMWARE] MANUAL state publish failed (rc={result.rc})")
    elif message.topic == "robot/cmd/drive":
        thr = data.get("thr")
        turn = data.get("turn")
        valid_drive = all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and -1 <= value <= 1
            for value in (thr, turn)
        )
        with state_lock:
            if robot_mode == "MANUAL" and valid_drive:
                drive_state["thr"] = float(thr)
                drive_state["turn"] = float(turn)
                print(f"[FAKE FIRMWARE] Drive accepted: {drive_state}")
            else:
                print("[FAKE FIRMWARE] Drive ignored while ESTOP is active or invalid")
    elif message.topic == "robot/cmd/arm":
        with state_lock:
            if robot_mode == "MANUAL" and data.get("action") in {"PICK", "DROP", "STOW"}:
                arm_state = data["action"]
                print(f"[FAKE FIRMWARE] Arm command accepted: {arm_state}")
            else:
                print("[FAKE FIRMWARE] Arm command ignored while ESTOP is active or invalid")


def main():
    global robot_mode, arm_state, outgoing_seq

    with state_lock:
        robot_mode = "ESTOP"
        drive_state.update(thr=0.0, turn=0.0)
        arm_state = "STOW"
    with sequence_lock:
        outgoing_seq = 0
    connected.clear()

    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        client_id=os.environ.get("ROV_MQTT_FAKE_CLIENT_ID", "fake_firmware_test"),
    )
    client.on_connect = on_connect
    client.on_message = on_message

    # Firmware online status is plain text, including the broker-published LWT.
    client.will_set(
        "robot/status/online",
        payload="offline",
        qos=1,
        retain=True,
    )

    client.connect(BROKER_HOST, BROKER_PORT)
    client.loop_start()

    if not connected.wait(timeout=5):
        client.disconnect()
        client.loop_stop()
        raise RuntimeError(
            f"Could not connect to MQTT broker at {BROKER_HOST}:{BROKER_PORT}."
        )

    print("[FAKE FIRMWARE] Running. Press Ctrl+C to stop.")

    telemetry_count = 0
    last_telemetry = 0.0
    last_heartbeat = 0.0

    try:
        while True:
            now = time.monotonic()

            # Send telemetry once per second on the canonical GUI topic.
            if now - last_telemetry >= 1.0:
                telemetry_count += 1

                with state_lock:
                    current_mode = robot_mode
                    current_arm = arm_state
                    current_drive = dict(drive_state)

                telemetry = {
                    "x": round(0.5 + telemetry_count * 0.01, 2),
                    "y": 1.2,
                    "theta": 0.3,
                    "v": current_drive["thr"],
                    "w": current_drive["turn"],
                    "mode": current_mode,
                    "arm": current_arm,
                    "batt_v": 7.4,
                    "pose_valid": True,
                    "link_ok": True,
                }

                publish_json(client, "robot/telemetry", telemetry, qos=0)
                print(f"[SENT] Telemetry #{telemetry_count}")
                last_telemetry = now

            # Send robot heartbeat every 0.2 seconds (target 5 Hz).
            if now - last_heartbeat >= 0.2:
                publish_json(
                    client,
                    "robot/hb/robot",
                    {"ts": int(time.time() * 1000)},
                    qos=0,
                )
                last_heartbeat = now

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[FAKE FIRMWARE] Stopping...")

    finally:
        print("[FAKE FIRMWARE] Disconnecting...")
        info = client.publish("robot/status/online", "offline", qos=1, retain=True)
        info.wait_for_publish(timeout=2.0)
        client.disconnect()
        client.loop_stop()
        print("[FAKE FIRMWARE] Disconnected.")


if __name__ == "__main__":
    main()

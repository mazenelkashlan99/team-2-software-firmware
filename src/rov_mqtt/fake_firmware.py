
import json
import time
from threading import Event, Lock
from uuid import uuid4

import paho.mqtt.client as mqtt

BROKER_HOST = "localhost"
BROKER_PORT = 1883

connected = Event()

# One session ID for this Fake Firmware session
SID = str(uuid4())

# One shared sequence counter for all outgoing application messages
outgoing_seq = 0
sequence_lock = Lock()


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


def on_connect(client, userdata, flags, reason_code, properties):
    print(f"[FAKE FIRMWARE] Connected: {reason_code}")

    if reason_code != 0:
        print("[FAKE FIRMWARE] Connection failed")
        return

    client.subscribe([
        ("robot/cmd/#", 1),
        ("robot/cv/#", 1),
        ("robot/hb/console", 0),
    ])

    publish_json(
        client,
        "robot/status/online",
        {"status": "online"},
        qos=1,
        retain=True,
    )

    publish_json(
        client,
        "robot/status/event",
        {
            "ev": "MISSION_LOADED",
            "detail": "Test mission loaded",
        },
        qos=1,
    )

    # Events are transient. Publish the current mission state separately and
    # retain it so a GUI that connects later can still render the active state.
    publish_json(
        client,
        "robot/state/mission",
        {
            "loaded": True,
            "detail": "Test mission loaded",
        },
        qos=1,
        retain=True,
    )

    print("[FAKE FIRMWARE] Subscriptions requested.")
    connected.set()


def on_message(client, userdata, message):
    payload = message.payload.decode("utf-8")

    print(f"\n[RECEIVED] {message.topic}")

    try:
        data = json.loads(payload)
        print(json.dumps(data, indent=2))
    except json.JSONDecodeError:
        print(payload)
        return

    if message.topic == "robot/cmd/ping" and isinstance(data, dict):
        pong = {
            "ping_id": data.get("ping_id"),
            "t_send": data.get("t_send"),
        }

        publish_json(
            client,
            "robot/state/pong",
            pong,
            qos=0,
        )

        print("[SENT] robot/state/pong")


client = mqtt.Client(
    callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
    client_id="fake_firmware_test",
)

client.on_connect = on_connect
client.on_message = on_message

# LWT: broker publishes this if the client disconnects unexpectedly.
# seq=0 is reserved for this special broker-generated message.
will_payload = json.dumps({
    "status": "offline",
    "ver": 1,
    "sid": SID,
    "seq": 0,
})

client.will_set(
    "robot/status/online",
    payload=will_payload,
    qos=1,
    retain=True,
)

client.connect(BROKER_HOST, BROKER_PORT)
client.loop_start()

if not connected.wait(timeout=5):
    client.loop_stop()
    raise RuntimeError("Could not connect to MQTT broker.")

print("[FAKE FIRMWARE] Running. Press Ctrl+C to stop.")

telemetry_count = 0
last_telemetry = 0.0
last_heartbeat = 0.0

try:
    while True:
        now = time.monotonic()

        # Send telemetry once per second
        if now - last_telemetry >= 1.0:
            telemetry_count += 1

            telemetry = {
                "x": round(0.5 + telemetry_count * 0.01, 2),
                "y": 1.2,
                "theta": 0.3,
                "v": 0.2,
                "w": 0.1,
                "mode": "MANUAL",
                "auto_state": "IDLE",
                "arm": "STOW",
                "batt_v": 7.4,
                "pose_valid": True,
                "link_ok": True,
            }

            publish_json(
                client,
                "robot/state/telemetry",
                telemetry,
                qos=0,
            )

            print(f"[SENT] Telemetry #{telemetry_count}")
            last_telemetry = now

        # Send robot heartbeat every 0.2 seconds (target 5 Hz)
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

    info = publish_json(
        client,
        "robot/status/online",
        {"status": "offline"},
        qos=1,
        retain=True,
    )

    info.wait_for_publish(timeout=2.0)

    client.disconnect()
    client.loop_stop()

    print("[FAKE FIRMWARE] Disconnected.")

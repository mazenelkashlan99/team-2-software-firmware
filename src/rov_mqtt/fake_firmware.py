import json
import time 
from threading import Event
import paho.mqtt.client as mqtt

BROKER_HOST="localhost"
BROKER_PORT =1883
connected =Event()

def on_connect(client,userdata,flags,reason_code,properties):
    print(f"[FAKE FIRMWARE] Connected: {reason_code}")

    if reason_code != 0:
        print("[FAKE FIRMWARE] Connection failed")
        return

    client.subscribe([
        ("robot/cmd/#",1),
        ("robot/cv/#",1),
        ("robot/hb/console",0),
    ])

    client.publish(
        "robot/status/online",
        "online",
        qos =1,
        retain=True,
    )

    client.publish(
        "robot/status/event",
        json.dumps({
            "ev": "MISSION_LOADED",
            "detail":"Test mission loaded",
        }),
        qos=1,
    )
    print(f"[FAKE FIRMWARE] Subscriptions requested.")
    connected.set()
def on_message(client,userdata,message):
    payload = message.payload.decode("utf-8")
    print(f"\n[RECIEVED] {message.topic}")
    try:
        data = json.loads(payload)
        print(json.dumps(data,indent=2))
    except json.JSONDecodeError:
        print(payload)
        return
    if message.topic == "robot/cmd/ping" and isinstance(data, dict):
        pong = {
            "ping_id": data.get("ping_id"),
            "t_send": data.get("t_send"),
        }

        client.publish(
            "robot/state/pong",
            json.dumps(pong),
            qos=0,
        )

        print("[SENT] robot/state/pong")
client = mqtt.Client(
    callback_api_version=mqtt.CallbackAPIVersion.VERSION2,client_id="fake_firmware_test",

)

client.on_connect = on_connect
client.on_message = on_message

client.will_set(                    #if the connection stopped send OFFLINE 
    "robot/status/online",
    payload="offline",
    qos=1,
    retain=True,
)

client.connect(BROKER_HOST, BROKER_PORT)
client.loop_start()

if not connected.wait(timeout=5):
    client.loop_stop()
    raise RuntimeError("Could not connect to MQTT brocker.")
print("[MOCK FIRMWARE] Running. Press Ctrl+C to stop.")

seq = 0
last_telemetry = 0.0
last_heartbeat = 0.0


try:
    while True:

        now = time.monotonic()

        if now - last_telemetry >= 1.0:
            seq += 1
            telemetry = {
                "x": round(0.5 + seq * 0.01, 2),
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
            client.publish(
                "robot/state/telemetry",
                json.dumps(telemetry),
                qos=0,
            )
            print(f"[SENT] Telemetry #{seq}")

            last_telemetry = now
        if now - last_heartbeat>= 0.2:
            client.publish(
                "robot/hb/robot",
                json.dumps({"seq":seq}),
                qos=0,
            )
            last_heartbeat = now

        time.sleep(0.02)


except KeyboardInterrupt:
    print("\n[MOCK FIRMWARE] Stopping...")
finally:
    print("[MOCK FIRMWARE] Disconnecting...")
    info = client.publish(
        "robot/status/online",
        payload="offline",
        qos=1,
        retain=True,
    )

    info.wait_for_publish(timeout=2.0)

    client.disconnect()
    client.loop_stop()

    print("[MOCK FIRMWARE] Disconnected.")

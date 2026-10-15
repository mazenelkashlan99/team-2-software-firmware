#!/usr/bin/env python3
"""Fake GUI: sends drive commands, heartbeats, mode and estop."""
import argparse
import json
import sys
import threading
import time

import paho.mqtt.client as mqtt

TOPIC_CMD_DRIVE = "robot/cmd/drive"
TOPIC_CMD_MODE  = "robot/cmd/mode"
TOPIC_CMD_ESTOP = "robot/cmd/estop"
TOPIC_CMD_RESET = "robot/cmd/estop_reset"
TOPIC_CMD_ARM   = "robot/cmd/arm"
TOPIC_CMD_PING  = "robot/cmd/ping"
TOPIC_HB_CONSOLE = "robot/heartbeat/console"
TOPIC_TELEM     = "robot/state/telemetry"
TOPIC_MODE      = "robot/state/mode"
TOPIC_EVENT     = "robot/status/event"


class Console:
    def __init__(self, host, port, sid):
        self.sid = sid
        self.seq = 0
        self.running = True
        self.client = mqtt.Client(client_id=f"fake-console-{sid}")
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.host = host
        self.port = port

    def _on_connect(self, client, userdata, flags, rc):
        print(f"[mqtt] connected rc={rc}")
        client.subscribe(TOPIC_TELEM)
        client.subscribe(TOPIC_MODE)
        client.subscribe(TOPIC_EVENT)

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8", errors="replace"))
        except Exception:
            payload = msg.payload
        if msg.topic == TOPIC_TELEM:
            print(f"[telem] mode={payload.get('mode')} v={payload.get('v')} w={payload.get('w')}")
        elif msg.topic == TOPIC_MODE:
            print(f"[mode]  -> {payload.get('mode')}")
        else:
            print(f"[{msg.topic}] {payload}")

    def _envelope(self, extra=None):
        self.seq += 1
        env = {"ver": 1, "sid": self.sid, "seq": self.seq}
        if extra:
            env.update(extra)
        return json.dumps(env)

    def publish(self, topic, extra=None):
        self.client.publish(topic, self._envelope(extra), qos=1)

    def start(self):
        self.client.connect(self.host, self.port, keepalive=30)
        self.client.loop_start()

        def hb():
            while self.running:
                self.client.publish(
                    TOPIC_HB_CONSOLE,
                    self._envelope({"ts": int(time.time() * 1000)}),
                    qos=0,
                )
                time.sleep(0.5)
        threading.Thread(target=hb, daemon=True).start()

    def stop(self):
        self.running = False
        self.client.loop_stop()
        self.client.disconnect()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--sid", default="gui")
    args = ap.parse_args()

    c = Console(args.host, args.port, args.sid)
    c.start()
    print("commands: w <thr> <turn> | a <PICK|DROP|STOW> | p <id> | m <MODE> | x | r | q")

    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()
            if cmd == "q":
                break
            elif cmd == "w" and len(parts) == 3:
                c.publish(TOPIC_CMD_DRIVE, {"thr": float(parts[1]), "turn": float(parts[2])})
                print(f"  -> sent drive thr={parts[1]} turn={parts[2]}")
            elif cmd == "m" and len(parts) == 2:
                c.publish(TOPIC_CMD_MODE, {"mode": parts[1].upper()})
                print(f"  -> sent mode {parts[1].upper()}")
            elif cmd == "a" and len(parts) == 2:
                action = parts[1].upper()
                if action not in ("PICK", "DROP", "STOW"):
                    print("  arm action must be PICK, DROP, or STOW")
                    continue
                c.publish(TOPIC_CMD_ARM, {"action": action})
                print(f"  -> sent arm {action}")
            elif cmd == "p" and len(parts) == 2:
                c.publish(TOPIC_CMD_PING, {"ping_id": int(parts[1])})
                print(f"  -> sent ping {parts[1]}")
            elif cmd == "x":
                c.publish(TOPIC_CMD_ESTOP)
                print("  -> sent ESTOP")
            elif cmd == "r":
                c.publish(TOPIC_CMD_RESET)
                print("  -> sent estop_reset")
            else:
                print("  usage: w <thr> <turn> | a <PICK|DROP|STOW> | p <id> | m <MODE> | x | r | q")
    except KeyboardInterrupt:
        pass
    finally:
        c.stop()


if __name__ == "__main__":
    main()

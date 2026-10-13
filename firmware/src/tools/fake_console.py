#!/usr/bin/env python3
"""Fake GUI: publish drive / mode / estop over MQTT and print telemetry.

A replacement for the real console while the software team's GUI isn't
ready. Publishes heartbeat at 2 Hz, lets you send drive commands from
stdin, and prints whatever arrives on robot/telem and robot/mode_state.

Commands from stdin:
    w 0.3 0        drive v=0.3, w=0
    m AUTO         request mode AUTO
    x              estop
    r              estop_reset
    q              quit

Usage:
    python fake_console.py --host localhost
    python fake_console.py --host 192.168.1.10 --sid test-pc
"""
import argparse
import json
import sys
import threading
import time

import paho.mqtt.client as mqtt


TOPIC_CMD_DRIVE      = "robot/cmd/drive"
TOPIC_CMD_MODE       = "robot/cmd/mode"
TOPIC_CMD_ESTOP      = "robot/cmd/estop"
TOPIC_CMD_ESTOP_RESET = "robot/cmd/estop_reset"
TOPIC_HB_CONSOLE     = "robot/heartbeat/console"
TOPIC_TELEM          = "robot/telem"
TOPIC_MODE_STATE     = "robot/mode_state"
TOPIC_EVENT          = "robot/event"


class Console:
    def __init__(self, host, port, sid):
        self.sid = sid
        self.seq = 0
        self.client = mqtt.Client(client_id=f"fake-console-{sid}")
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.host = host
        self.port = port
        self.running = True

    def _on_connect(self, client, userdata, flags, rc):
        print(f"[mqtt] connected rc={rc}")
        client.subscribe(TOPIC_TELEM)
        client.subscribe(TOPIC_MODE_STATE)
        client.subscribe(TOPIC_EVENT)

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8", errors="replace"))
        except Exception:
            payload = msg.payload
        print(f"[{msg.topic}] {payload}")

    def _envelope(self, extra: dict) -> str:
        self.seq += 1
        msg = {"ver": 1, "sid": self.sid, "seq": self.seq,
               "t": int(time.time() * 1000)}
        msg.update(extra)
        return json.dumps(msg)

    def publish(self, topic, extra):
        self.client.publish(topic, self._envelope(extra), qos=1)

    def start(self):
        self.client.connect(self.host, self.port, keepalive=30)
        self.client.loop_start()

        # Heartbeat thread.
        def hb():
            while self.running:
                self.client.publish(TOPIC_HB_CONSOLE,
                                    self._envelope({}), qos=0)
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
    ap.add_argument("--sid", default="fake-console")
    args = ap.parse_args()

    c = Console(args.host, args.port, args.sid)
    c.start()
    print("commands: w <v> <w> | m <MODE> | x | r | q")

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
                c.publish(TOPIC_CMD_DRIVE, {"thr": float(parts[1]),
                                            "turn": float(parts[2])})
            elif cmd == "m" and len(parts) == 2:
                c.publish(TOPIC_CMD_MODE, {"mode": parts[1].upper()})
            elif cmd == "x":
                c.publish(TOPIC_CMD_ESTOP, {})
            elif cmd == "r":
                c.publish(TOPIC_CMD_ESTOP_RESET, {})
            else:
                print("unknown command")
    except KeyboardInterrupt:
        pass
    finally:
        c.stop()


if __name__ == "__main__":
    main()
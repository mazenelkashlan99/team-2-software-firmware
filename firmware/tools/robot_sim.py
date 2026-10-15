#!/usr/bin/env python3
import argparse
import json
import time

import paho.mqtt.client as mqtt

TOPIC_SUB_CMD   = "robot/cmd/drive"
TOPIC_SUB_HB    = "robot/heartbeat/console"
TOPIC_SUB_MODE  = "robot/cmd/mode"
TOPIC_SUB_ESTOP = "robot/cmd/estop"
TOPIC_SUB_RESET = "robot/cmd/estop_reset"
TOPIC_TELEM     = "robot/telem"
TOPIC_MODE      = "robot/mode_state"
TOPIC_HB_ROBOT  = "robot/heartbeat/robot"
TOPIC_STATUS    = "robot/status"
TOPIC_DIAG      = "robot/diagnostics"

SID = "robot-sim-ahmed"
HEARTBEAT_TIMEOUT_MS = 500
CMD_TIMEOUT_MS       = 500
MAX_V_MPS            = 0.3
MAX_W_RADPS          = 1.0


class RobotSim:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.seq = 0
        self.mode = "ESTOP"
        self.last_hb_ms = 0
        self.last_cmd_ms = 0
        self.target_v = 0.0
        self.target_w = 0.0
        self.wheelL = 0.0
        self.wheelR = 0.0
        self.start_time = int(time.time())
        self.client = mqtt.Client(client_id=SID)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.will_set(TOPIC_STATUS, "offline", qos=1, retain=True)

    def _envelope(self):
        self.seq += 1
        return {"ver": 1, "sid": SID, "seq": self.seq, "t": int(time.time() * 1000)}

    def _publish(self, topic, extra=None):
        env = self._envelope()
        if extra:
            env.update(extra)
        self.client.publish(topic, json.dumps(env))

    def _on_connect(self, client, userdata, flags, rc):
        print(f"[sim] connected rc={rc}")
        client.subscribe(TOPIC_SUB_CMD, qos=1)
        client.subscribe(TOPIC_SUB_HB, qos=0)
        client.subscribe(TOPIC_SUB_MODE, qos=1)
        client.subscribe(TOPIC_SUB_ESTOP, qos=1)
        client.subscribe(TOPIC_SUB_RESET, qos=1)
        client.publish(TOPIC_HB_ROBOT, json.dumps(self._envelope()))
        client.publish(TOPIC_STATUS, "online", qos=1, retain=True)

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            print(f"[sim] bad json on {topic}")
            return
        if payload.get("ver") != 1:
            print(f"[sim] bad version on {topic}: {payload.get('ver')}")
            return
        if topic == TOPIC_SUB_HB:
            self.last_hb_ms = int(time.time() * 1000)
            return
        if topic == TOPIC_SUB_CMD:
            thr = float(payload.get("thr", 0.0))
            turn = float(payload.get("turn", 0.0))
            self.target_v = thr * MAX_V_MPS
            self.target_w = turn * MAX_W_RADPS
            self.last_cmd_ms = int(time.time() * 1000)
            print(f"[sim] DRIVE thr={thr:+.2f} turn={turn:+.2f}")
        elif topic == TOPIC_SUB_MODE:
            new_mode = payload.get("mode", "MANUAL")
            if new_mode in ("MANUAL", "SEMI", "AUTO"):
                self.mode = new_mode
                self._publish(TOPIC_MODE, {"mode": self.mode})
                print(f"[sim] MODE -> {self.mode}")
        elif topic == TOPIC_SUB_ESTOP:
            self.mode = "ESTOP"
            self.target_v = 0.0
            self.target_w = 0.0
            self._publish(TOPIC_MODE, {"mode": "ESTOP"})
            print("[sim] ESTOP latched")
        elif topic == TOPIC_SUB_RESET:
            now = int(time.time() * 1000)
            if now - self.last_hb_ms < HEARTBEAT_TIMEOUT_MS:
                self.mode = "MANUAL"
                self._publish(TOPIC_MODE, {"mode": self.mode})
                print("[sim] RESET -> MANUAL")
            else:
                print("[sim] RESET refused: no fresh heartbeat")

    def run(self):
        self.client.connect(self.host, self.port, keepalive=30)
        self.client.loop_start()
        last_telem = 0
        last_hb = 0
        last_diag = 0
        try:
            while True:
                now = int(time.time() * 1000)
                if now - self.last_cmd_ms > CMD_TIMEOUT_MS:
                    self.target_v = 0.0
                    self.target_w = 0.0
                if now - self.last_hb_ms > HEARTBEAT_TIMEOUT_MS and self.mode != "ESTOP":
                    self.mode = "ESTOP"
                    self._publish(TOPIC_MODE, {"mode": "ESTOP"})
                    print("[sim] watchdog -> ESTOP")
                self.wheelL += (self.target_v - self.wheelL) * 0.2
                self.wheelR += (self.target_v - self.wheelR) * 0.2
                if now - last_telem >= 100:
                    last_telem = now
                    self._publish(TOPIC_TELEM, {
                        "x": 0.0, "y": 0.0, "theta": 0.0,
                        "v": round((self.wheelL + self.wheelR) * 0.5, 3),
                        "w": round((self.wheelR - self.wheelL) / 0.15, 3),
                        "mode": self.mode,
                        "auto_state": "IDLE",
                        "arm": "IDLE",
                        "batt_v": 7.4,
                        "pose_valid": True,
                        "link_ok": True,
                    })
                if now - last_hb >= 200:
                    last_hb = now
                    self.client.publish(TOPIC_HB_ROBOT, json.dumps(self._envelope()))
                if now - last_diag >= 1000:
                    last_diag = now
                    self._publish(TOPIC_DIAG, {
                        "uptime_s": int(time.time()) - self.start_time,
                        "heap_free": 234567,
                        "wifi_rssi": -42,
                        "loop_us": 982,
                        "batt_v": 7.4,
                        "batt_pct": 82,
                    })
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.client.publish(TOPIC_STATUS, "offline", qos=1, retain=True)
            time.sleep(0.2)
            self.client.loop_stop()
            self.client.disconnect()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    args = ap.parse_args()
    RobotSim(args.host, args.port).run()


if __name__ == "__main__":
    main()

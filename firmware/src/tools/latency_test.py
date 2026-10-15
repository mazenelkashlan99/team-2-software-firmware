#!/usr/bin/env python3
"""Measure round-trip latency console -> broker -> robot -> broker -> console.

Publishes robot/cmd/ping with a unique ping_id at a set rate, listens
for robot/state/pong, and reports the distribution of round-trip times.

Usage:
    python latency_test.py --host localhost --count 100 --rate 10
"""
import argparse
import json
import statistics
import time

import paho.mqtt.client as mqtt


TOPIC_PING = "robot/cmd/ping"
TOPIC_PONG = "robot/state/pong"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--sid", default="latency-test")
    ap.add_argument("--count", type=int, default=100)
    ap.add_argument("--rate", type=float, default=10.0, help="pings per second")
    ap.add_argument("--timeout", type=float, default=1.0)
    args = ap.parse_args()

    sent = {}          # ping_id -> send time
    rtts = []
    seq = 0

    client = mqtt.Client(client_id=f"latency-{args.sid}")

    def on_connect(c, u, flags, rc):
        print(f"[mqtt] connected rc={rc}")
        c.subscribe(TOPIC_PONG)

    def on_message(c, u, msg):
        try:
            p = json.loads(msg.payload.decode())
        except Exception:
            return
        pid = p.get("ping_id")
        if pid in sent:
            rtt = (time.time() - sent.pop(pid)) * 1000.0
            rtts.append(rtt)
            print(f"\rping {pid} rtt {rtt:.1f} ms", end="")

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, args.port, 30)
    client.loop_start()
    time.sleep(0.5)

    interval = 1.0 / args.rate
    t_start = time.time()
    for i in range(args.count):
        seq += 1
        pid = int(time.time() * 1000) % 1_000_000 + i
        payload = {"ver": 1, "sid": args.sid, "seq": seq, "t": pid,
                   "ping_id": pid}
        sent[pid] = time.time()
        client.publish(TOPIC_PING, json.dumps(payload), qos=0)
        time.sleep(interval)

    # Wait for stragglers.
    t_end = time.time() + args.timeout
    while sent and time.time() < t_end:
        time.sleep(0.05)

    client.loop_stop()
    client.disconnect()

    print()
    if not rtts:
        print("no pongs received; check firmware is publishing robot/state/pong")
        return
    rtts.sort()
    print(f"received: {len(rtts)} / {args.count}")
    print(f"min:  {rtts[0]:.1f} ms")
    print(f"med:  {statistics.median(rtts):.1f} ms")
    print(f"p95:  {rtts[int(0.95 * len(rtts)) - 1]:.1f} ms")
    print(f"max:  {rtts[-1]:.1f} ms")


if __name__ == "__main__":
    main()

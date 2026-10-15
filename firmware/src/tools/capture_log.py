#!/usr/bin/env python3
"""Read serial lines from the robot and write them to a CSV.

Firmware is expected to emit one sample per line, as space-separated
key=value pairs:

    t=1234 sp=0.300 v=0.281 p=0.452

Lines starting with '#' are treated as comments and written verbatim
into a sidecar .txt file. Empty lines are skipped.

Usage:
    python capture_log.py --port /dev/ttyUSB0 --out logs/motor_step.csv
    python capture_log.py --port COM5 --baud 115200 --out run1.csv --duration 10
"""
import argparse
import csv
import re
import sys
import time
from pathlib import Path

import serial


KV_RE = re.compile(r"(\w+)=([^\s]+)")


def parse_line(line: str):
    """Return (keys_in_order, values_by_key) or None if no kv pairs."""
    pairs = KV_RE.findall(line)
    if not pairs:
        return None
    keys = [k for k, _ in pairs]
    vals = {k: v for k, v in pairs}
    return keys, vals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyUSB0 or COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", required=True, help="CSV output path")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds; 0 means run until Ctrl-C")
    ap.add_argument("--flush-every", type=int, default=20,
                    help="flush CSV to disk every N rows")
    args = ap.parse_args()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    comments = out.with_suffix(".txt")

    print(f"opening {args.port} @ {args.baud}")
    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    time.sleep(2.0)   # ESP32 resets on serial open; wait for boot
    ser.reset_input_buffer()

    t0 = time.time()
    rows = 0
    header = None
    f_csv = open(out, "w", newline="")
    f_txt = open(comments, "w")
    writer = None
    try:
        while True:
            if args.duration > 0 and (time.time() - t0) > args.duration:
                break
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            if not line:
                continue
            if line.startswith("#"):
                f_txt.write(line + "\n")
                continue
            parsed = parse_line(line)
            if parsed is None:
                f_txt.write(f"# unparsed: {line}\n")
                continue
            keys, vals = parsed
            if header is None:
                header = keys
                writer = csv.DictWriter(f_csv, fieldnames=header, extrasaction="ignore")
                writer.writeheader()
            # Rows may miss a key; fill missing with empty.
            writer.writerow({k: vals.get(k, "") for k in header})
            rows += 1
            if rows % args.flush_every == 0:
                f_csv.flush()
                print(f"\r{rows} rows", end="", file=sys.stderr)
    except KeyboardInterrupt:
        print("\nstopped by user", file=sys.stderr)
    finally:
        f_csv.close()
        f_txt.close()
        ser.close()
    print(f"\nwrote {rows} rows -> {out}")


if __name__ == "__main__":
    main()
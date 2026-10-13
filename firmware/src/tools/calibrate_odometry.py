#!/usr/bin/env python3
"""Calibrate ticks_per_meter and track_width from two ground-truth runs.

You drive the robot by hand (or with a fixed command) and record:
  - straight line of measured length L_true (tape measure)
  - 10 in-place turns and record the final heading change (gyro or
    visual reference)

The CSV should include cumulative distance left/right (meters) and
yaw (radians), as reported by the firmware. This script computes the
correction factors to apply to config.h.

Usage:
    python calibrate_odometry.py straight.csv --true-distance 2.0
    python calibrate_odometry.py spin.csv --true-yaw-deg 360 --track-width 0.15
"""
import argparse
import numpy as np
import pandas as pd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--true-distance", type=float, default=None)
    ap.add_argument("--true-yaw-deg", type=float, default=None)
    ap.add_argument("--track-width", type=float, default=0.15,
                    help="current configured trackWidth_ (m)")
    ap.add_argument("--ticks-per-meter", type=float, default=None,
                    help="current configured TICKS_PER_METER")
    args = ap.parse_args()

    df = pd.read_csv(args.csv).sort_values("t")
    if "totalDist" in df.columns:
        measured_dist = float(df["totalDist"].iloc[-1] - df["totalDist"].iloc[0])
    elif "distL" in df.columns and "distR" in df.columns:
        dL = float(df["distL"].iloc[-1] - df["distL"].iloc[0])
        dR = float(df["distR"].iloc[-1] - df["distR"].iloc[0])
        measured_dist = 0.5 * (dL + dR)
    else:
        print("need totalDist or distL/distR columns")
        return

    if args.true_distance is not None:
        scale = measured_dist / args.true_distance
        print(f"measured distance: {measured_dist:.4f} m")
        print(f"true distance:     {args.true_distance:.4f} m")
        print(f"distance scale error: {(scale-1.0)*100:+.2f} %")
        if args.ticks_per_meter:
            corrected = args.ticks_per_meter * scale
            print(f"new TICKS_PER_METER: {corrected:.2f}  (was {args.ticks_per_meter:.2f})")

    if args.true_yaw_deg is not None:
        if "theta" not in df.columns:
            print("need theta column for yaw calibration")
            return
        measured_yaw = float(df["theta"].iloc[-1] - df["theta"].iloc[0])
        expected = np.deg2rad(args.true_yaw_deg)
        ratio = measured_yaw / expected
        print()
        print(f"measured yaw: {np.rad2deg(measured_yaw):.2f} deg")
        print(f"true yaw:     {args.true_yaw_deg:.2f} deg")
        print(f"yaw scale error: {(ratio-1.0)*100:+.2f} %")
        # theta = dR - dL integrated; if measured is too small, trackWidth
        # is too large. Correct: new_track = old_track / ratio.
        new_track = args.track_width / ratio
        print(f"new TRACK_WIDTH_M: {new_track:.4f}  (was {args.track_width:.4f})")


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""Analyze a straight-line test.

Expects CSV with columns t, x, y, theta, v (m/s, m, m, rad).
Reports:
  - total distance traveled
  - final (x, y)
  - heading drift (max |theta|, std of theta)
  - speed overshoot and settling time to the commanded setpoint

Usage:
    python analyze_straight.py logs/straight_1m.csv --target 1.0
"""
import argparse
import numpy as np
import pandas as pd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--target", type=float, default=None,
                    help="commanded v (m/s); enables settling-time report")
    args = ap.parse_args()

    df = pd.read_csv(args.csv).dropna(subset=["t", "x", "y", "theta"]).sort_values("t")
    t   = df["t"].to_numpy(float) / 1000.0
    x   = df["x"].to_numpy(float)
    y   = df["y"].to_numpy(float)
    th  = df["theta"].to_numpy(float)

    dx = np.diff(x)
    dy = np.diff(y)
    dist = float(np.sum(np.hypot(dx, dy)))

    print(f"samples: {len(df)}")
    print(f"duration: {t[-1] - t[0]:.2f} s")
    print(f"total distance: {dist:.3f} m")
    print(f"final x: {x[-1]:.3f} m")
    print(f"final y: {y[-1]:.3f} m")
    print(f"heading drift (max |theta|): {np.max(np.abs(th)):.4f} rad")
    print(f"heading std: {np.std(th):.4f} rad")

    if "v" in df.columns and args.target:
        v = df["v"].to_numpy(float)
        peak = float(np.max(v))
        overshoot = (peak - args.target) / args.target * 100.0
        # Settling time: last time |v - target| > 5% of target.
        band = 0.05 * abs(args.target)
        outside = np.abs(v - args.target) > band
        if outside.any():
            last_out = int(np.max(np.where(outside)[0]))
            settle_t = t[last_out] - t[0]
        else:
            settle_t = 0.0
        print(f"v peak: {peak:.3f} m/s, overshoot: {overshoot:.1f} %")
        print(f"settling time (5%): {settle_t:.3f} s")


if __name__ == "__main__":
    main()
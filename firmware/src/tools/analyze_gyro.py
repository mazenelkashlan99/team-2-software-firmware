#!/usr/bin/env python3
"""Analyze a gyro capture.

Two modes:
  --still   robot stationary; compute bias, noise std
  --rotate  robot rotated by a known angle; check scale

Expects CSV with column gz (rad/s) and t (ms).

Usage:
    python analyze_gyro.py logs/gyro_still.csv --still
    python analyze_gyro.py logs/gyro_90deg.csv --rotate --angle-deg 90
"""
import argparse
import numpy as np
import pandas as pd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--still", action="store_true")
    ap.add_argument("--rotate", action="store_true")
    ap.add_argument("--angle-deg", type=float, default=None,
                    help="true rotation for --rotate")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    df = pd.read_csv(args.csv).dropna(subset=["t", "gz"]).sort_values("t")
    t  = df["t"].to_numpy(float) / 1000.0
    gz = df["gz"].to_numpy(float)

    if args.still:
        bias = float(np.mean(gz))
        std  = float(np.std(gz))
        print(f"samples: {len(gz)}")
        print(f"gyro bias: {bias*1000:.4f} mrad/s ({bias:.6f} rad/s)")
        print(f"noise std: {std*1000:.4f} mrad/s")
        if args.plot:
            import matplotlib.pyplot as plt
            plt.plot(t, gz * 1000.0)
            plt.axhline(bias * 1000.0, color="red", ls="--")
            plt.xlabel("time (s)")
            plt.ylabel("gz (mrad/s)")
            plt.grid(True)
            plt.show()
        return

    if args.rotate:
        if args.angle_deg is None:
            print("--rotate requires --angle-deg")
            return
        # Trapezoidal integration of gz over time gives integrated angle.
        integrated = np.trapz(gz, t)   # rad
        expected = np.deg2rad(args.angle_deg)
        scale_err = (integrated - expected) / expected * 100.0
        print(f"integrated angle: {np.rad2deg(integrated):.2f} deg")
        print(f"expected angle:  {args.angle_deg:.2f} deg")
        print(f"scale error: {scale_err:+.2f} %")
        if args.plot:
            import matplotlib.pyplot as plt
            plt.plot(t, np.concatenate(([0], np.cumsum(gz[:-1] * np.diff(t)))) * 180.0 / np.pi)
            plt.axhline(args.angle_deg, color="red", ls="--")
            plt.xlabel("time (s)")
            plt.ylabel("integrated yaw (deg)")
            plt.grid(True)
            plt.show()
        return

    print("specify --still or --rotate")


if __name__ == "__main__":
    main()
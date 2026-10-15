#!/usr/bin/env python3
"""Find motor deadband from a slow power ramp log.

Expects a CSV with columns t (ms), u (commanded power), v (measured speed).
Finds the smallest |u| at which |v| exceeds a movement threshold.

Usage:
    python find_deadband.py logs/deadband.csv
    python find_deadband.py logs/deadband.csv --v-thresh 0.02 --plot
"""
import argparse
import numpy as np
import pandas as pd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--v-thresh", type=float, default=0.01,
                    help="speed considered 'moving' (m/s)")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    df = pd.read_csv(args.csv).dropna(subset=["t", "u", "v"]).sort_values("t")
    u = df["u"].to_numpy(float)
    v = df["v"].to_numpy(float)

    moving = np.abs(v) > args.v_thresh
    if not moving.any():
        print("never moved above threshold; increase power or lower --v-thresh")
        return

    first = np.argmax(moving)
    print(f"first movement at index {first}, u={u[first]:.3f}, v={v[first]:.3f}")

    # Average the tiny window around first motion for a more stable estimate.
    lo = max(0, first - 3)
    hi = min(len(u), first + 4)
    deadband = float(np.mean(np.abs(u[lo:hi])))
    print(f"estimated deadband: {deadband:.4f}")

    if args.plot:
        import matplotlib.pyplot as plt
        plt.plot(u, v)
        plt.axvline(deadband, color="red", ls="--", label=f"db={deadband:.3f}")
        plt.xlabel("commanded power")
        plt.ylabel("measured speed (m/s)")
        plt.grid(True)
        plt.legend()
        plt.show()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""Plot x-y trajectory from a telemetry or pose log.

Usage:
    python plot_path.py logs/run.csv
    python plot_path.py logs/run.csv --targets 1.5,0.5 2.0,1.2 --save path.png
"""
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--targets", nargs="*", default=[],
                    help="targets as x,y pairs, e.g. 1.5,0.5 2.0,1.2")
    ap.add_argument("--save", default=None)
    args = ap.parse_args()

    df = pd.read_csv(args.csv).dropna(subset=["x", "y"]).sort_values("t")
    x = df["x"].to_numpy(float)
    y = df["y"].to_numpy(float)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(x, y, "-", label="path")
    ax.plot(x[0], y[0], "go", label="start")
    ax.plot(x[-1], y[-1], "ro", label="end")

    for tgt in args.targets:
        tx, ty = map(float, tgt.split(","))
        ax.plot(tx, ty, "k*", ms=12)
        ax.annotate(f"({tx},{ty})", (tx, ty), textcoords="offset points",
                    xytext=(6, 6))

    # If theta is present, draw heading arrows every N samples.
    if "theta" in df.columns:
        th = df["theta"].to_numpy(float)
        step = max(1, len(x) // 20)
        for i in range(0, len(x), step):
            ax.arrow(x[i], y[i], 0.05 * np.cos(th[i]), 0.05 * np.sin(th[i]),
                     head_width=0.02, color="C1", alpha=0.6)

    ax.set_aspect("equal", "box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True)
    ax.legend()
    if args.save:
        plt.savefig(args.save, dpi=120, bbox_inches="tight")
        print(f"saved -> {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
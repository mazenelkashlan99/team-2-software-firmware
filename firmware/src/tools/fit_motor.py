#!/usr/bin/env python3
"""Fit a first-order motor model from a step-response log.

Model:
    v_dot = (K * u - v) / tau
where v is wheel speed (m/s), u is commanded power in [-1, 1], K is the
steady-state gain (m/s per unit power), and tau is the time constant.

Outputs:
    K, tau
    kff = 1 / K       (feed-forward gain for the PID)
    suggested PI gains from IMC tuning with a chosen closed-loop
    bandwidth (default 10 rad/s):
        kp = tau * omega_c / K
        ki = kp / (tau * 2)          # aggressive; halve if it oscillates

Usage:
    python fit_motor.py logs/motor_step.csv
    python fit_motor.py logs/motor_step.csv --omega-c 20 --plot
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import curve_fit


def first_order_step(t, K, tau, t0):
    """Step response of a first-order system starting at t0."""
    y = np.zeros_like(t)
    m = t >= t0
    y[m] = K * (1.0 - np.exp(-(t[m] - t0) / tau))
    return y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="CSV with columns t, u, v (seconds, power, speed)")
    ap.add_argument("--omega-c", type=float, default=10.0,
                    help="target closed-loop bandwidth (rad/s)")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--t0", type=float, default=None,
                    help="time the step was applied; default = first sample with u > 0")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    df = df.dropna(subset=["t", "u", "v"]).sort_values("t")
    t = df["t"].to_numpy(dtype=float) / 1000.0   # ms -> s if needed
    u = df["u"].to_numpy(dtype=float)
    v = df["v"].to_numpy(dtype=float)

    # Detect the step if not given.
    if args.t0 is None:
        idx = np.argmax(np.abs(u) > 0.05)
        t0 = t[idx]
        u_step = u[idx]
    else:
        t0 = args.t0
        u_step = u[np.argmin(np.abs(t - t0))]

    print(f"step detected at t={t0:.3f} s, u={u_step:.3f}")

    # Fit only the portion after the step.
    m = t >= t0
    t_fit = t[m] - t0
    v_fit = v[m]

    p0 = [max(v_fit.max(), 0.01) / max(abs(u_step), 0.01), 0.05, 0.0]

    def model(t_, K, tau, t_off):
        return first_order_step(t_ + t0, K, tau, t0 + t_off) * u_step

    try:
        (K, tau, t_off), pcov = curve_fit(model, t_fit, v_fit, p0=p0, maxfev=10000)
    except Exception as e:
        print(f"fit failed: {e}")
        return

    print(f"K   = {K:.4f} m/s per unit power")
    print(f"tau = {tau*1000:.1f} ms")
    print(f"fit offset = {t_off*1000:.2f} ms")

    kff = 1.0 / K
    kp  = tau * args.omega_c / K
    ki  = kp / (tau * 2.0)
    print()
    print("Suggested PID gains:")
    print(f"  kff = {kff:.3f}")
    print(f"  kp  = {kp:.3f}")
    print(f"  ki  = {ki:.3f}")

    if args.plot:
        import matplotlib.pyplot as plt
        plt.plot(t, v, label="measured")
        plt.plot(t[m], model(t_fit, K, tau, t_off), "--", label="fit")
        plt.axvline(t0, color="gray", lw=0.5)
        plt.xlabel("time (s)")
        plt.ylabel("speed (m/s)")
        plt.legend()
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    main()
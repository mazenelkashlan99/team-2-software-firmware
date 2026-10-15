#!/usr/bin/env python3
"""Simulate a first-order motor + discrete PID. Plots step response.

The PID matches the firmware: feed-forward term, integral clamped in
power units, derivative on measurement, output clamped to [-1, 1].

Usage:
    python sim_pid.py --K 1.2 --tau 0.08 --kp 0.5 --ki 4 --kd 0 --kff 0.83
    python sim_pid.py --K 1.2 --tau 0.08 --auto --omega-c 10
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt


def simulate(K, tau, kp, ki, kd, kff, sp, dt=0.01, t_end=2.0,
             out_min=-1.0, out_max=1.0, int_min=-0.5, int_max=0.5):
    n = int(t_end / dt)
    t = np.arange(n) * dt
    v = 0.0
    integral = 0.0
    prev_meas = 0.0
    first = True
    v_hist = np.zeros(n)
    u_hist = np.zeros(n)
    for i in range(n):
        err = sp - v
        d = 0.0 if first else (v - prev_meas) / dt
        prev_meas = v
        first = False
        integral += ki * err * dt
        integral = max(int_min, min(int_max, integral))
        u = kff * sp + kp * err + integral - kd * d
        u = max(out_min, min(out_max, u))
        # Motor: first-order lag.
        v += (K * u - v) * dt / tau
        v_hist[i] = v
        u_hist[i] = u
    return t, v_hist, u_hist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--K", type=float, required=True)
    ap.add_argument("--tau", type=float, required=True)
    ap.add_argument("--kp", type=float, default=0.0)
    ap.add_argument("--ki", type=float, default=0.0)
    ap.add_argument("--kd", type=float, default=0.0)
    ap.add_argument("--kff", type=float, default=0.0)
    ap.add_argument("--sp", type=float, default=0.3)
    ap.add_argument("--auto", action="store_true",
                    help="derive gains from K, tau, --omega-c")
    ap.add_argument("--omega-c", type=float, default=10.0)
    args = ap.parse_args()

    if args.auto:
        args.kff = 1.0 / args.K
        args.kp  = args.tau * args.omega_c / args.K
        args.ki  = args.kp / (args.tau * 2.0)
        args.kd  = 0.0
        print(f"auto gains: kff={args.kff:.3f} kp={args.kp:.3f} ki={args.ki:.3f}")

    t, v, u = simulate(args.K, args.tau, args.kp, args.ki, args.kd, args.kff, args.sp)

    fig, ax = plt.subplots(2, 1, sharex=True)
    ax[0].plot(t, v, label="v (m/s)")
    ax[0].axhline(args.sp, color="gray", ls="--", label="setpoint")
    ax[0].set_ylabel("speed (m/s)")
    ax[0].legend()
    ax[0].grid(True)
    ax[1].plot(t, u, color="C1")
    ax[1].set_ylabel("power")
    ax[1].set_xlabel("time (s)")
    ax[1].grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
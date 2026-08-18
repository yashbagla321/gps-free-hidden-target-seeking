#!/usr/bin/env python3
"""Render paper figures from the GPS-free seeking simulation CSVs.

Usage: python3 scripts/render_figures.py [results_dir]
Writes PDF (for LaTeX) and PNG (for quick viewing) into results_dir.
"""

import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

RESULTS = Path(sys.argv[1] if len(sys.argv) > 1 else "results")
TARGET = (12.0, 8.0)
BEACON = (6.0, -4.0)


def read(name):
    with open(RESULTS / name) as f:
        return list(csv.DictReader(f))


def save(fig, stem):
    for ext in ("pdf", "png"):
        fig.savefig(RESULTS / f"{stem}.{ext}", dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {RESULTS / stem}.pdf/.png")


def fig_nominal():
    rows = read("s1_nominal_timeseries.csv")
    t = [float(r["t"]) for r in rows]
    qx = [float(r["qx"]) for r in rows]
    qy = [float(r["qy"]) for r in rows]
    dist = [float(r["dist"]) for r in rows]
    psi = [float(r["psi_err"]) for r in rows]
    ee = [float(r["e_hat_err"]) for r in rows]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.0, 3.4))
    ax1.plot(qx, qy, "-", color="tab:blue", lw=1.2, label="vehicle path")
    ax1.plot(qx[0], qy[0], "ko", ms=5, label="start")
    ax1.plot(*TARGET, "r*", ms=13, label="hidden target $p$")
    ax1.plot(*BEACON, "g^", ms=9, label="relay beacon $x$ (pose unknown)")
    ax1.set_xlabel("$X$ [m]")
    ax1.set_ylabel("$Y$ [m]")
    ax1.legend(fontsize=7, loc="lower right")
    ax1.set_title("(a) closed-loop trajectory, no GPS", fontsize=9)
    ax1.axis("equal")
    ax1.grid(alpha=0.3)

    ax2.semilogy(t, dist, label=r"$\|q-p\|$ vehicle-to-target")
    ax2.semilogy(t, ee, label=r"$\|\hat e - e\|$ relative estimate error")
    ax2.semilogy(t, [max(v, 1e-4) for v in psi], label=r"$|\tilde\psi|$ yaw error [rad]")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("error")
    ax2.legend(fontsize=7)
    ax2.set_title("(b) convergence", fontsize=9)
    ax2.grid(alpha=0.3, which="both")
    save(fig, "fig_nominal")


def fig_noise():
    rows = read("s2_noise_mc.csv")
    s = [float(r["noise_scale"]) for r in rows]
    med = [float(r["median_final_dist"]) for r in rows]
    q90 = [float(r["q90_final_dist"]) for r in rows]
    fig, ax = plt.subplots(figsize=(4.6, 3.2))
    ax.plot(s, med, "o-", label="median final distance")
    ax.plot(s, q90, "s--", label="90th percentile")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xticks(s)
    ax.set_xticklabels([str(v) for v in s])
    ax.set_xlabel(r"beacon noise scale (base: $\sigma_r=0.1$ m, $\sigma_\beta=1^\circ$)")
    ax.set_ylabel("station-keeping error [m]")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, which="both")
    save(fig, "fig_noise")


def fig_gps_sweep():
    rows = read("s3_gps_sweep.csv")
    methods = {
        "proposed_no_gps": ("proposed (uses no GPS)", "tab:blue", "o-"),
        "ekf_gps": ("absolute EKF + GPS", "tab:orange", "s-"),
        "naive_gps_as_truth": ("GPS treated as truth", "tab:red", "^-"),
    }
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.0, 3.2))
    for key, (label, color, style) in methods.items():
        pts = [(float(r["sigma_gps"]), float(r["median_final_dist"]))
               for r in rows if r["method"] == key]
        pts.sort()
        ax1.plot([p[0] for p in pts], [p[1] for p in pts], style, color=color,
                 label=label)
    ax1.set_xscale("log")
    ax1.set_yscale("log")
    ax1.set_xlabel(r"GPS noise $\sigma_{\rm gps}$ [m]")
    ax1.set_ylabel("median vehicle-to-target distance [m]")
    ax1.set_title("(a) task error", fontsize=9)
    ax1.legend(fontsize=7)
    ax1.grid(alpha=0.3, which="both")

    pts = [(float(r["sigma_gps"]), float(r["median_abs_target_err"]))
           for r in rows if r["method"] == "ekf_gps"]
    pts.sort()
    ax2.plot([p[0] for p in pts], [p[1] for p in pts], "s-", color="tab:orange",
             label="absolute EKF + GPS")
    ax2.set_xscale("log")
    ax2.set_xlabel(r"GPS noise $\sigma_{\rm gps}$ [m]")
    ax2.set_ylabel(r"absolute target error $\|\hat p - p\|$ [m]")
    ax2.set_title("(b) absolute estimate (only defined with GPS)", fontsize=9)
    ax2.legend(fontsize=7)
    ax2.grid(alpha=0.3, which="both")
    save(fig, "fig_gps_sweep")


def fig_drift():
    rows = read("s5_drift_timeseries.csv")
    t = [float(r["t"]) for r in rows]
    dist = [float(r["dist"]) for r in rows]
    dr = [float(r["deadreck_err"]) for r in rows]
    sweep = read("s5_drift_bias_sweep.csv")
    b = [float(r["odo_bias"]) for r in sweep]
    med = [float(r["median_final_dist"]) for r in sweep]
    q90 = [float(r["q90_final_dist"]) for r in sweep]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.0, 3.2))
    ax1.semilogy(t, dist, color="tab:blue",
                 label=r"task error $\|q-p\|$ (proposed)")
    ax1.semilogy(t, dr, color="tab:gray",
                 label="dead-reckoned position error")
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("error [m]")
    ax1.set_title("(a) 600 s station keeping, odometry bias 2.2 cm/s", fontsize=9)
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3, which="both")

    ax2.plot(b, med, "o-", label="median")
    ax2.plot(b, q90, "s--", label="90th percentile")
    ax2.set_xlabel("odometry bias magnitude [m/s]")
    ax2.set_ylabel("station-keeping error [m]")
    ax2.set_title("(b) bias sweep (600 s horizon)", fontsize=9)
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)
    save(fig, "fig_drift")


def fig_excitation():
    rows = read("s4_excitation_ablation.csv")
    fig, ax = plt.subplots(figsize=(4.6, 3.2))
    for exc, label, color in ((1, "with decaying excitation", "tab:blue"),
                              (0, "no injected excitation", "tab:red")):
        tt = sorted(float(r["time_to_reach"]) for r in rows
                    if int(r["excitation"]) == exc and r["reached"] == "1")
        n = len(tt)
        ax.step(tt, [i / n for i in range(1, n + 1)], where="post",
                color=color, label=f"{label} ({n}/100 reached)")
    ax.set_xlabel("time to reach 0.5 m [s]")
    ax.set_ylabel("empirical CDF")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    save(fig, "fig_excitation")


if __name__ == "__main__":
    fig_nominal()
    fig_noise()
    fig_gps_sweep()
    fig_drift()
    fig_excitation()

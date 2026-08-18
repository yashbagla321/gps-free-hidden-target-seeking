#!/usr/bin/env python3
"""Generates EVERY figure in the campaign 2027 manuscript from the
provenance-locked clean run. Single reproducible entry point.

Usage:
    python3 scripts/paper_figures.py [--run RUN_DIR] [--out OUT_DIR]

Representative-trajectory panels use visualization traces produced by
bin/viz_dump (built from the same commit); they are illustrative runs,
never the source of any statistic in the paper.
"""

import argparse
import csv
import os
import subprocess
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DEF_RUN = ("results/campaign2027/offline/"
           "run_212a98b29759e3e4f21e29c33341a47c3da9c80c")


def rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def col(rs, k):
    return np.array([float(r[k]) for r in rs])


def save(fig, out, stem):
    fig.savefig(os.path.join(out, stem + ".pdf"), bbox_inches="tight")
    fig.savefig(os.path.join(out, stem + ".png"), dpi=170,
                bbox_inches="tight")
    plt.close(fig)
    print("wrote", stem)


def fig_certificate(run, out):
    rs = rows(f"{run}/s2_odometry_coverage.csv")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 2.5))
    markers = {"8": "o", "16": "s"}
    for nv in ("8", "16"):
        sel = [r for r in rs if r["n_views"] == nv]
        ax1.loglog(col(sel, "pred_var"), col(sel, "emp_mse"),
                   markers[nv], ms=5, label=f"$K={nv}$")
    lims = [col(rs, "pred_var").min() * 0.7, col(rs, "pred_var").max() * 1.4]
    ax1.loglog(lims, lims, "k-", lw=0.8, label="ideal")
    ax1.set_xlabel("predicted variance [rad$^2$]")
    ax1.set_ylabel("empirical variance [rad$^2$]")
    ax1.legend(fontsize=6, loc="upper left", framealpha=0.9,
               handlelength=1.2, borderpad=0.3, labelspacing=0.25)
    ax1.grid(alpha=0.3, which="both")
    ax1.set_title("(a) predicted vs empirical", fontsize=9)
    for nv in ("8", "16"):
        sel = [r for r in rs if r["n_views"] == nv]
        ax2.semilogx(col(sel, "sigma_xy_step_m") * 100, col(sel, "cov95"),
                     markers[nv] + "-", ms=4, lw=1.0, label=f"$K={nv}$")
    ax2.axhline(0.95, color="k", lw=0.8)
    ax2.axhspan(0.95 - 0.0096, 0.95 + 0.0096, color="gray", alpha=0.22,
                label="Wilson, $n{=}2000$")
    ax2.set_ylim(0.90, 1.0)
    ax2.set_xlabel("translation noise per increment [cm]")
    ax2.set_ylabel("realized 95% coverage")
    ax2.legend(fontsize=5.5, loc="lower left", ncol=3, framealpha=0.85,
               handlelength=1.1, borderpad=0.25, labelspacing=0.2,
               columnspacing=0.7, handletextpad=0.4)
    ax2.grid(alpha=0.3, which="both")
    ax2.set_title("(b) coverage across odometry noise", fontsize=9)
    save(fig, out, "fig_certificate")


def fig_longhorizon(run, out):
    rs = rows(f"{run}/s6_drift_timeseries.csv")
    fig, ax = plt.subplots(figsize=(3.4, 2.2))
    ax.semilogy(col(rs, "t"), np.maximum(col(rs, "deadreck_err"), 1e-3),
                color="tab:gray", label="dead-reckoned pose error")
    ax.semilogy(col(rs, "t"), np.maximum(col(rs, "dist"), 1e-3),
                color="tab:blue", label="task error (proposed)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("error [m]")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3, which="both")
    save(fig, out, "fig_longhorizon")


def ensure_traces(out):
    tdir = os.path.join(out, "traces")
    os.makedirs(tdir, exist_ok=True)
    nom = os.path.join(tdir, "viz_nominal.csv")
    yst = os.path.join(tdir, "viz_yawstep.csv")
    if not (os.path.exists(nom) and os.path.exists(yst)):
        subprocess.run(["g++", "-O2", "-std=c++17", "-Iinclude",
                        "src/viz_dump.cpp", "-o", "bin/viz_dump"], check=True)
        subprocess.run(["./bin/viz_dump", nom, "nominal", "12"], check=True)
        subprocess.run(["./bin/viz_dump", yst, "yawstep", "12"], check=True)
    return nom, yst


def fig_trajectories(run, out):
    nom_p, yst_p = ensure_traces(out)
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.9))
    for ax, path, title in ((axes[0], nom_p, "(a) nominal mission"),
                            (axes[1], yst_p,
                             "(b) relay rotated $60^\\circ$ at $t=8$ s")):
        rs = rows(path)
        qx, qy = col(rs, "qx"), col(rs, "qy")
        ax.plot(col(rs, "drx"), col(rs, "dry"), color="0.65", lw=0.9,
                label="dead reckoning")
        ax.plot(qx, qy, color="tab:blue", lw=1.5, label="vehicle (true)")
        modes = col(rs, "mode")
        exc = modes == 0
        ax.plot(qx[exc], qy[exc], ".", color="tab:orange", ms=2.5,
                label="EXCITE phase")
        ax.plot(float(rs[0]["px"]), float(rs[0]["py"]), "r*", ms=13,
                label="hidden target")
        ax.plot(float(rs[0]["rx"]), float(rs[0]["ry"]), "g^", ms=9,
                label="relay (pose unknown)")
        ax.axis("equal")
        ax.grid(alpha=0.3)
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("X [m]")
    axes[0].set_ylabel("Y [m]")
    axes[0].legend(fontsize=6, loc="best")
    save(fig, out, "fig_trajectories")


def fig_robustness(run, out):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 2.4))
    comm = rows(f"{run}/s8_communication.csv")
    styles = {"dropout": ("o-", "dropout [frac.]"),
              "delay_s": ("s-", "delay [s]"),
              "jitter_s": ("d-", "jitter [s]"),
              "outlier_prob": ("^-", "outliers [frac.]")}
    for axis, (st, lab) in styles.items():
        sel = [r for r in comm if r["axis"] == axis and float(r["value"]) > 0]
        ax1.plot(col(sel, "value"), col(sel, "median_rmse"), st, ms=4,
                 lw=1.1, label=lab)
    ax1.set_xlabel("degradation level")
    ax1.set_ylabel("median station RMSE [m]")
    ax1.set_ylim(0, 0.25)
    ax1.legend(fontsize=6, loc="upper left")
    ax1.grid(alpha=0.3)
    ax1.set_title("(a) communication stress, all $200/200$", fontsize=9)

    relay = rows(f"{run}/s4_relay_noise.csv")
    rng = [r for r in relay if r["axis"] == "range"]
    brg = [r for r in relay if r["axis"] == "bearing_deg"]
    ax2.plot(col(rng, "sigma") * 10, col(rng, "median_rmse"), "o-", ms=4,
             lw=1.1, label="range noise [dm]")
    ax2.plot(col(brg, "sigma"), col(brg, "median_rmse"), "s-", ms=4,
             lw=1.1, label="bearing noise [deg]")
    ax2.set_xscale("log")
    ax2.set_xlabel("noise level")
    ax2.set_ylabel("median station RMSE [m]")
    ax2.set_ylim(0, 0.25)
    ax2.legend(fontsize=6, loc="upper left")
    ax2.grid(alpha=0.3, which="both")
    ax2.set_title("(b) relay noise, all $200/200$", fontsize=9)
    save(fig, out, "fig_robustness")


def fig_disturbance(run, out):
    tr = [r for r in rows(f"{run}/s7_disturbance_transit.csv")
          if r["variant"] == "supervised"]
    st = [r for r in rows(f"{run}/s7_disturbance_station.csv")
          if r["variant"] == "supervised"]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 2.4))
    for sel, mk, lab in ((tr, "o-", "mid-transit step"),
                         (st, "s-", "at-rest step")):
        d = col(sel, "delta_deg")
        rr = col(sel, "recovery_rate_required")
        lo = col(sel, "recovery_wilson_lo")
        hi = col(sel, "recovery_wilson_hi")
        ax1.errorbar(d, rr, yerr=[rr - lo, hi - rr], fmt=mk, ms=4,
                     capsize=3, lw=1.1, label=lab)
    ax1.set_ylim(0, 1.05)
    ax1.set_xlabel("relay yaw step $\\Delta$ [deg]")
    ax1.set_ylabel("calibration recovery rate")
    ax1.legend(fontsize=7, loc="lower right")
    ax1.grid(alpha=0.3)
    ax1.set_title("(a) recovery is threshold sharp", fontsize=9)

    for sel, mk, lab in ((tr, "o-", "mid-transit step"),
                         (st, "s-", "at-rest step")):
        ax2.plot(col(sel, "delta_deg"),
                 np.degrees(col(sel, "median_theta_err")), mk, ms=4, lw=1.1,
                 label=lab)
    ax2.axhspan(0, 10, color="tab:green", alpha=0.15,
                label="certified band")
    ax2.set_xlabel("relay yaw step $\\Delta$ [deg]")
    ax2.set_ylabel("terminal yaw error [deg]")
    ax2.legend(fontsize=7, loc="upper right")
    ax2.grid(alpha=0.3)
    ax2.set_title("(b) task success $150/150$ throughout", fontsize=9)
    save(fig, out, "fig_disturbance")


FIG_STEMS = ["fig_certificate", "fig_longhorizon", "fig_trajectories",
             "fig_robustness", "fig_disturbance"]
PACKAGES = ["docs/campaign2027_submission/source", "docs/campaign2027_submission/arxiv"]
BUILDS = [("docs/campaign2027_submission/source", "main"),
          ("docs/campaign2027_submission/arxiv", "main_arxiv")]


def deploy(out):
    """Copy every figure PDF into both manuscript source packages so the
    central figures directory and the packages can never diverge."""
    import shutil
    for pkg in PACKAGES:
        for stem in FIG_STEMS:
            shutil.copy(os.path.join(out, stem + ".pdf"), pkg)
        print("deployed figures ->", pkg)


def build_and_hash():
    """Rebuild both manuscripts (two pdflatex passes each) and print the
    SHA-256 of every deployed figure and produced PDF for verification."""
    import hashlib
    for d, stem in BUILDS:
        for _ in range(2):
            subprocess.run(["pdflatex", "-interaction=nonstopmode",
                            stem + ".tex"], cwd=d,
                           stdout=subprocess.DEVNULL, check=True)
        print("built", os.path.join(d, stem + ".pdf"))
    print("--- SHA-256 ---")
    for d, stem in BUILDS:
        for f in sorted(os.listdir(d)):
            if f.endswith(".pdf"):
                h = hashlib.sha256(
                    open(os.path.join(d, f), "rb").read()).hexdigest()
                print(f"{h}  {d}/{f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default=DEF_RUN)
    ap.add_argument("--out", default="docs/campaign2027_submission/figures")
    ap.add_argument("--no-build", action="store_true",
                    help="generate and deploy figures without rebuilding PDFs")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    fig_certificate(a.run, a.out)
    fig_longhorizon(a.run, a.out)
    fig_trajectories(a.run, a.out)
    fig_robustness(a.run, a.out)
    fig_disturbance(a.run, a.out)
    deploy(a.out)
    if not a.no_build:
        build_and_hash()


if __name__ == "__main__":
    sys.exit(main())

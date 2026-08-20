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
import glob
import os
import subprocess
import sys

import matplotlib

matplotlib.use("Agg")
# Type 1/42 (TrueType), not the matplotlib default Type 3: some venue
# submission checks reject Type 3 fonts embedded in figure PDFs.
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import NullLocator, ScalarFormatter

DEF_RUN = ("results/campaign2027/offline/"
           "run_212a98b29759e3e4f21e29c33341a47c3da9c80c")
# Covariance-ablation study (s9_) was added after DEF_RUN was captured;
# it lives in its own clean provenance-locked run rather than requiring a
# full re-run of every other study (whose default-path numbers Phase 3's
# additive changes do not affect).
DEF_COV_RUN = ("results/campaign2027/offline/"
               "run_20c493e349176321969ef3143298ba5f4df1af36")
DEF_GZ_RUN = ("results/campaign2027/ros_gz/citable/"
              "0af43095d347c3ca46e4fcfec34d9ecc4165f208/campaign2027_final")


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


def _draw_certificate(rs, figsize, fs_axis, fs_tick, fs_legend, fs_title,
                       ms):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=figsize)
    markers = {"8": "o", "16": "s"}
    for nv in ("8", "16"):
        sel = [r for r in rs if r["n_views"] == nv]
        ax1.loglog(col(sel, "pred_var"), col(sel, "emp_mse"),
                   markers[nv], ms=ms, label=f"$K={nv}$")
    pv_min, pv_max = col(rs, "pred_var").min(), col(rs, "pred_var").max()
    lims = [pv_min * 0.7, pv_max * 1.4]
    ax1.loglog(lims, lims, "k-", lw=0.8, label="ideal")
    ax1.set_xlabel("predicted variance [rad$^2$]", fontsize=fs_axis)
    ax1.set_ylabel("empirical variance [rad$^2$]", fontsize=fs_axis)
    ax1.tick_params(axis="both", labelsize=fs_tick)
    # Extra headroom beyond the plotted data/ideal-line extent so the
    # upper-right legend sits in clear space instead of overlapping the
    # top of the diagonal trend.
    ax1.set_xlim(pv_min * 0.5, pv_max * 4.0)
    ax1.set_ylim(pv_min * 0.5, pv_max * 4.0)
    ax1.legend(fontsize=fs_legend, loc="upper right", framealpha=0.9,
               handlelength=1.2, borderpad=0.3, labelspacing=0.25)
    ax1.grid(alpha=0.3, which="both")
    ax1.set_title("(a) predicted vs empirical", fontsize=fs_title)
    for nv in ("8", "16"):
        sel = [r for r in rs if r["n_views"] == nv]
        ax2.semilogx(col(sel, "sigma_xy_step_m") * 100, col(sel, "cov95"),
                     markers[nv] + "-", ms=ms - 0.5, lw=1.0, label=f"$K={nv}$")
    ax2.axhline(0.95, color="k", lw=0.8)
    ax2.axhspan(0.95 - 0.0096, 0.95 + 0.0096, color="gray", alpha=0.22,
                label="Wilson, $n{=}2000$")
    ax2.set_ylim(0.90, 1.0)
    ax2.set_xlabel("translation noise per increment [cm]", fontsize=fs_axis)
    ax2.set_ylabel("realized 95% coverage", fontsize=fs_axis)
    ax2.legend(fontsize=fs_legend, loc="upper right", ncol=1, framealpha=0.9,
               handlelength=1.1, borderpad=0.3, labelspacing=0.25)
    # Explicit ticks at the tested noise levels only: the default log-scale
    # minor-tick locator crowds in extra labels (2x, 3x, ...) that overlap
    # when the span is under one decade.
    xticks = sorted(set(col(rs, "sigma_xy_step_m") * 100))
    ax2.xaxis.set_minor_locator(NullLocator())
    ax2.set_xticks(xticks)
    ax2.xaxis.set_major_formatter(ScalarFormatter())
    ax2.tick_params(axis="both", labelsize=fs_tick)
    ax2.grid(alpha=0.3, which="both")
    ax2.set_title("(b) coverage across odometry noise", fontsize=fs_title)
    return fig


def fig_certificate(run, out):
    """Single-column, two-panel: matches the figsize/font-size convention
    of fig_robustness/fig_covariance_ablation/fig_disturbance (generated
    at double the physical print size for a single-column placement).
    Matches the page-limited conference variant of the manuscript."""
    rs = rows(f"{run}/s2_odometry_coverage.csv")
    fig = _draw_certificate(rs, figsize=(7.0, 2.6), fs_axis=8, fs_tick=7,
                            fs_legend=6.5, fs_title=9, ms=4)
    save(fig, out, "fig_certificate")


def fig_certificate_wide(run, out):
    """Double-column variant at larger absolute font sizes for the
    page-unlimited extended manuscript, which can afford a wider, more
    legible rendering of the same underlying data."""
    rs = rows(f"{run}/s2_odometry_coverage.csv")
    fig = _draw_certificate(rs, figsize=(14.0, 5.0), fs_axis=11, fs_tick=10,
                            fs_legend=9, fs_title=12, ms=5)
    save(fig, out, "fig_certificate_wide")


def _draw_longhorizon(rs, figsize, fs_legend):
    fig, ax = plt.subplots(figsize=figsize)
    ax.semilogy(col(rs, "t"), np.maximum(col(rs, "deadreck_err"), 1e-3),
                color="tab:gray", label="dead-reckoned pose error")
    ax.semilogy(col(rs, "t"), np.maximum(col(rs, "dist"), 1e-3),
                color="tab:blue", label="task error (proposed)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("error [m]")
    ax.legend(fontsize=fs_legend)
    ax.grid(alpha=0.3, which="both")
    return fig


def fig_longhorizon(run, out):
    """Deployed at width=0.62\\linewidth in the page-limited conference
    variant (narrower than fig_certificate/fig_robustness's \\linewidth),
    so the canvas is scaled down to the same fraction to keep text the
    same print size as those figures rather than shrinking with the
    smaller placement."""
    rs = rows(f"{run}/s6_drift_timeseries.csv")
    fig = _draw_longhorizon(rs, figsize=(4.3, 2.8), fs_legend=7)
    save(fig, out, "fig_longhorizon")


def fig_longhorizon_wide(run, out):
    """Deployed at width=\\linewidth in the page-unlimited extended
    manuscript: the doubled-canvas convention shared with
    fig_robustness/fig_disturbance (generated at roughly double the
    physical print size for a single-column placement)."""
    rs = rows(f"{run}/s6_drift_timeseries.csv")
    fig = _draw_longhorizon(rs, figsize=(6.8, 4.4), fs_legend=7)
    save(fig, out, "fig_longhorizon_wide")


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
    ax1.legend(fontsize=6, loc="upper right")
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
    ax2.set_ylim(0, 0.25)
    ax2.legend(fontsize=6, loc="upper right")
    # Explicit ticks at the tested noise levels only (see fig_certificate):
    # avoids overlapping auto-generated log-scale minor tick labels.
    xticks = sorted(set(col(rng, "sigma") * 10) | set(col(brg, "sigma")))
    ax2.xaxis.set_minor_locator(NullLocator())
    ax2.set_xticks(xticks)
    ax2.xaxis.set_major_formatter(ScalarFormatter())
    ax2.tick_params(axis="x", labelsize=7)
    ax2.grid(alpha=0.3, which="both")
    ax2.set_title("(b) relay noise, all $200/200$", fontsize=9)
    save(fig, out, "fig_robustness")


def fig_covariance_ablation(run, out):
    """Phase 3 review-response figure: pooled variance ratio and realized
    95% coverage for the three certificate variance models (full, diag,
    packet_only) from the covariance-ablation study (s9_)."""
    rs = rows(f"{run}/s9_covariance_ablation_summary.csv")
    order = ["full", "diag", "packet_only"]
    labels = {"full": "full\n(proposed)", "diag": "diag\n(naive)",
              "packet_only": "packet-only\n(naive)"}
    by_model = {r["model"]: r for r in rs}
    ratios = [float(by_model[m]["pooled_ratio"]) for m in order]
    cov = [float(by_model[m]["pooled_cov95"]) for m in order]
    colors = ["tab:blue", "tab:orange", "tab:red"]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 2.4))
    ax1.bar(range(3), ratios, color=colors)
    ax1.axhline(1.0, color="k", lw=0.8, ls="--")
    ax1.set_yscale("log")
    ax1.set_xticks(range(3))
    ax1.set_xticklabels([labels[m] for m in order], fontsize=7)
    ax1.set_ylabel("empirical/predicted var. ratio")
    ax1.grid(alpha=0.3, axis="y", which="both")
    ax1.set_title("(a) pooled variance ratio", fontsize=9)

    ax2.bar(range(3), cov, color=colors)
    ax2.axhline(0.95, color="k", lw=0.8, ls="--", label="nominal $95\\%$")
    ax2.set_ylim(0, 1.0)
    ax2.set_xticks(range(3))
    ax2.set_xticklabels([labels[m] for m in order], fontsize=7)
    ax2.set_ylabel("realized 95% coverage")
    ax2.legend(fontsize=6, loc="lower left")
    ax2.grid(alpha=0.3, axis="y")
    ax2.set_title("(b) pooled coverage", fontsize=9)
    save(fig, out, "fig_covariance_ablation")


def _gazebo_metric(gz_run, family, key):
    values = []
    pattern = os.path.join(gz_run, family, "run_*.csv.summary")
    for path in sorted(glob.glob(pattern)):
        fields = {}
        with open(path) as f:
            for token in f.read().split():
                if "=" in token:
                    name, value = token.split("=", 1)
                    fields[name] = value
        if fields.get("complete") == "1" and key in fields:
            values.append(float(fields[key]))
    if not values:
        raise RuntimeError(f"no valid {key} values found for {family} in {gz_run}")
    return np.asarray(values)


def fig_operating_envelope(run, cov_run, gz_run, out):
    """Extended-manuscript-only four-panel view of calibration and mission
    boundaries."""
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 7.2),
                             constrained_layout=True)
    ax1, ax2, ax3, ax4 = axes.ravel()

    # Per-cell certificate coverage, not only the pooled bars in the paper.
    cov = rows(f"{cov_run}/s9_covariance_ablation.csv")
    colors = {"full": "tab:blue", "diag": "tab:orange",
              "packet_only": "tab:red"}
    names = {"full": "full", "diag": "diag",
             "packet_only": "packet-only"}
    for model in ("full", "diag", "packet_only"):
        for n_views, ls in ((8, "-"), (16, "--")):
            sel = sorted((r for r in cov if r["model"] == model and
                          int(r["n_views"]) == n_views),
                         key=lambda r: float(r["sigma_xy_step_m"]))
            ax1.plot(col(sel, "sigma_xy_step_m") * 100,
                     col(sel, "cov95"), marker="o", ms=4, lw=1.3, ls=ls,
                     color=colors[model], label=f"{names[model]}, K={n_views}")
    ax1.axhline(0.95, color="k", lw=0.9, ls=":", label="nominal 95%")
    ax1.set_xscale("log")
    ax1.set_ylim(0.15, 1.02)
    ax1.set_xlabel("translation noise per increment [cm]")
    ax1.set_ylabel("realized 95% coverage")
    ax1.set_title("(a) coverage requires cross-view correlation")
    ax1.grid(alpha=0.25, which="both")
    ax1.legend(fontsize=7, ncol=3, loc="upper left",
               bbox_to_anchor=(0.0, -0.22), borderaxespad=0,
               frameon=False, handlelength=1.8, columnspacing=1.0)

    # Closed-loop acquisition transition as odometry noise crosses the gate.
    odo = [r for r in rows(f"{run}/s5_odometry_proposed.csv")
           if r["axis"] == "sigma_xy"]
    odo.sort(key=lambda r: float(r["value"]))
    x_odo = col(odo, "value") * 100
    success = col(odo, "success_rate")
    ax2.semilogx(x_odo, success, "o-", color="tab:blue", lw=1.5, ms=5)
    ax2.axvline(2.0, color="tab:red", lw=1.0, ls="--")
    ax2.axvspan(2.0, x_odo.max() * 1.08, color="tab:red", alpha=0.10,
                label="certificate refuses")
    for x, y, r in zip(x_odo, success, odo):
        ax2.annotate(f"{int(round(float(r['n']) * y))}/{r['n']}",
                     (x, y), xytext=(0, 7), textcoords="offset points",
                     ha="center", fontsize=8)
    ax2.set_ylim(-0.04, 1.08)
    ax2.set_xlabel("translation noise per increment [cm]")
    ax2.set_ylabel("mission success rate")
    ax2.set_title("(b) honest refusal at the odometry boundary")
    ax2.grid(alpha=0.25, which="both")
    ax2.legend(fontsize=8, loc="center right")

    # Paired errors isolate estimator effects from randomized geometry.
    trials = rows(f"{cov_run}/s3_baselines_trials_all.csv")
    by_method = {}
    for row in trials:
        by_method.setdefault(row["method"], {})[row["seed"]] = row
    proposed = by_method["proposed"]
    paired = []
    labels = []
    omitted = []
    for method, label in (("oracle_yaw", "oracle - proposed"),
                          ("ekf", "EKF - proposed"),
                          ("endpoint_only", "endpoint - proposed")):
        common = sorted(set(proposed) & set(by_method[method]))
        successful = [s for s in common
                      if float(proposed[s]["success"]) > 0.5 and
                      float(by_method[method][s]["success"]) > 0.5]
        paired.append(np.asarray([
            float(by_method[method][s]["station_rmse"]) -
            float(proposed[s]["station_rmse"]) for s in successful]))
        labels.append(label)
        omitted.append(len(common) - len(successful))
    central_lo = min(np.quantile(values, 0.01) for values in paired)
    central_hi = max(np.quantile(values, 0.95) for values in paired)
    displayed = [values[(values >= central_lo) & (values <= central_hi)]
                 for values in paired]
    off_scale = [len(values) - len(shown)
                 for values, shown in zip(paired, displayed)]
    vp = ax3.violinplot(displayed, showmedians=True, showextrema=False)
    for body, color in zip(vp["bodies"],
                           ("tab:green", "tab:orange", "tab:purple")):
        body.set_facecolor(color)
        body.set_edgecolor("black")
        body.set_alpha(0.65)
    vp["cmedians"].set_color("black")
    ax3.axhline(0, color="k", lw=0.9, ls=":")
    ax3.set_xticks(range(1, len(labels) + 1))
    ax3.set_xticklabels(labels)
    ax3.set_ylabel("paired station-RMSE difference [m]")
    ax3.set_title("(c) paired estimator consequences")
    ax3.grid(alpha=0.25, axis="y")
    for i, (failed, tails) in enumerate(zip(omitted, off_scale), start=1):
        notes = []
        if failed:
            notes.append(f"{failed} failures")
        if tails:
            notes.append(f"{tails} tails")
        if notes:
            ax3.text(i, 0.98, ", ".join(notes),
                     transform=ax3.get_xaxis_transform(), ha="center",
                     va="top", fontsize=7)

    # Gazebo distributions expose run-to-run physics variation hidden by medians.
    families = ("nominal", "stress", "disturbance_transit")
    gz_values = [_gazebo_metric(gz_run, family, "station_rmse")
                 for family in families]
    bp = ax4.boxplot(gz_values, patch_artist=True, showfliers=False,
                     medianprops={"color": "black", "linewidth": 1.4})
    for box, color in zip(bp["boxes"],
                          ("tab:blue", "tab:orange", "tab:green")):
        box.set_facecolor(color)
        box.set_alpha(0.45)
    for i, values in enumerate(gz_values, start=1):
        offsets = np.linspace(-0.12, 0.12, len(values))
        ax4.plot(i + offsets, values, ".", color="0.25", ms=3.5, alpha=0.65)
    ax4.set_xticks((1, 2, 3))
    ax4.set_xticklabels(("nominal", "comm. stress", "60° step"))
    ax4.set_ylabel("station RMSE [m]")
    ax4.set_title("(d) Gazebo distributions, 30 launches each")
    ax4.grid(alpha=0.25, axis="y")

    save(fig, out, "fig_operating_envelope")


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default=DEF_RUN)
    ap.add_argument("--cov-run", default=DEF_COV_RUN,
                    help="run directory for the s9_ covariance-ablation "
                         "study (separate from --run; see DEF_COV_RUN)")
    ap.add_argument("--gz-run", default=DEF_GZ_RUN,
                    help="citable Gazebo campaign directory")
    ap.add_argument("--out", default="results/figures")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    fig_certificate(a.run, a.out)
    fig_certificate_wide(a.run, a.out)
    fig_longhorizon(a.run, a.out)
    fig_longhorizon_wide(a.run, a.out)
    fig_trajectories(a.run, a.out)
    fig_robustness(a.run, a.out)
    fig_disturbance(a.run, a.out)
    fig_covariance_ablation(a.cov_run, a.out)
    fig_operating_envelope(a.run, a.cov_run, a.gz_run, a.out)
    print("all figures written to", a.out)


if __name__ == "__main__":
    sys.exit(main())

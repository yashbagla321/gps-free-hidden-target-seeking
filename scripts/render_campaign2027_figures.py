#!/usr/bin/env python3
"""Render the figures used by the campaign 2027 manuscript from citable CSVs."""

import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_csv(path: Path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def wilson_interval(success_probability: float, n: int, z: float = 1.96):
    denominator = 1.0 + z * z / n
    center = (success_probability + z * z / (2.0 * n)) / denominator
    radius = z * math.sqrt(
        success_probability * (1.0 - success_probability) / n
        + z * z / (4.0 * n * n)
    ) / denominator
    return center - radius, center + radius


def render_certificate(results_dir: Path, output_path: Path):
    rows = read_csv(results_dir / "s2_odometry_coverage.csv")
    styles = {
        8: ("$K=8$ views", "#1f77b4", "o"),
        16: ("$K=16$ views", "#d55e00", "s"),
    }

    plt.rcParams.update({
        "font.size": 9.5,
        "axes.labelsize": 9.5,
        "axes.titlesize": 9.5,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "legend.fontsize": 8.0,
        "lines.linewidth": 1.3,
        "lines.markersize": 5.0,
    })

    fig, (ax_pred, ax_cov) = plt.subplots(1, 2, figsize=(7.0, 2.35))
    noise_levels = sorted({float(row["sigma_xy_step_m"]) for row in rows})
    noise_positions = list(range(len(noise_levels)))
    for n_views, (label, color, marker) in styles.items():
        group = [row for row in rows if int(row["n_views"]) == n_views]
        group.sort(key=lambda row: float(row["sigma_xy_step_m"]))
        ax_pred.loglog(
            [float(row["pred_var"]) for row in group],
            [float(row["emp_mse"]) for row in group],
            marker=marker,
            color=color,
            label=label,
        )
        ax_cov.plot(
            noise_positions,
            [float(row["cov95"]) for row in group],
            marker=marker,
            color=color,
            label=label,
        )

    values = [
        float(row[key])
        for row in rows
        for key in ("pred_var", "emp_mse")
    ]
    lower, upper = min(values) * 0.82, max(values) * 1.22
    ax_pred.plot([lower, upper], [lower, upper], "k--", label="ideal")
    ax_pred.set_xlim(lower, upper)
    ax_pred.set_ylim(lower, upper)
    ax_pred.set_xlabel(r"predicted yaw variance [rad$^2$]")
    ax_pred.set_ylabel(r"empirical yaw variance [rad$^2$]")
    ax_pred.set_title("(a) Predicted versus empirical")
    ax_pred.grid(alpha=0.3, which="both")
    ax_pred.legend(loc="upper left")

    n_realizations = int(rows[0]["n_realizations"])
    wilson_lo, wilson_hi = wilson_interval(0.95, n_realizations)
    x_min, x_max = -0.25, len(noise_positions) - 0.75
    ax_cov.fill_between(
        [x_min, x_max],
        [wilson_lo, wilson_lo],
        [wilson_hi, wilson_hi],
        color="0.85",
        label=f"95% Wilson band, $n={n_realizations}$",
        zorder=0,
    )
    ax_cov.axhline(0.95, color="k", linestyle="--", linewidth=1.0)
    ax_cov.set_xlim(x_min, x_max)
    ax_cov.set_ylim(0.925, 0.965)
    ax_cov.set_xticks(noise_positions)
    ax_cov.set_xticklabels(["0.25", "0.5", "1", "2", "5"])
    ax_cov.set_xlabel("translation noise per increment [cm]")
    ax_cov.set_ylabel("realized 95% coverage")
    ax_cov.set_title("(b) Coverage across odometry noise")
    ax_cov.grid(alpha=0.3, which="both")
    ax_cov.legend(loc="lower right")

    fig.tight_layout(w_pad=2.0)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {output_path}")


def main():
    if len(sys.argv) != 3:
        raise SystemExit(
            "usage: render_campaign2027_figures.py RESULTS_DIR OUTPUT_PDF"
        )
    render_certificate(Path(sys.argv[1]), Path(sys.argv[2]))


if __name__ == "__main__":
    main()

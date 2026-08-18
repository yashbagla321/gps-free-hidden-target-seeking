#!/usr/bin/env python3
"""Aggregate only provenance-matched, complete Gazebo campaign results."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

CONTRACT_DIR = (Path(__file__).resolve().parents[1] / "ros2_ws" / "src" /
                "gps_free_seeking_gz" / "scripts")
sys.path.insert(0, str(CONTRACT_DIR))
from gz_result_contract import ResultContractError, validate_run  # noqa: E402


FAMILIES = (
    "nominal", "stress", "disturbance", "disturbance_transit",
    "target_relocation",
)


def wilson_lb(k: int, n: int, z: float = 1.96) -> float:
    if n == 0:
        return float("nan")
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    r = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (c - r) / d


def quantiles(xs: list[float]) -> tuple[float, float, float]:
    xs = sorted(xs)
    if not xs:
        return (float("nan"),) * 3

    def q(fraction: float) -> float:
        index = fraction * (len(xs) - 1)
        lo, hi = math.floor(index), math.ceil(index)
        return xs[lo] + (xs[hi] - xs[lo]) * (index - lo)

    return q(0.25), q(0.5), q(0.75)


def distribution(xs: list[float], digits: int) -> dict | None:
    if not xs:
        return None
    q1, median, q3 = quantiles(xs)
    return {
        "median": round(median, digits),
        "iqr": [round(q1, digits), round(q3, digits)],
        "min": round(min(xs), digits),
        "max": round(max(xs), digits),
    }


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def sustained_recovery(rows: list[dict[str, float]], event_t: float,
                       predicate, horizon_s: float = 10.0) -> float | None:
    post = [row for row in rows if row["t"] >= event_t]
    if not post:
        return None
    final_t = post[-1]["t"]
    for row in post:
        if row["t"] + horizon_s > final_t + 1e-9:
            break
        horizon = [candidate for candidate in post
                   if row["t"] <= candidate["t"] <= row["t"] + horizon_s]
        if horizon and predicate(row) and all(predicate(candidate)
                                           for candidate in horizon):
            return row["t"] - event_t
    return None


def result_pairs(directory: Path) -> list[tuple[Path, Path, Path]]:
    csvs = {path.stem: path for path in directory.glob("run_*.csv")}
    summaries = {
        path.name.removesuffix(".csv.summary"): path
        for path in directory.glob("run_*.csv.summary")
    }
    metadata = {
        path.name.removesuffix(".meta.json"): path
        for path in directory.glob("run_*.meta.json")
    }
    if set(csvs) != set(summaries) or set(csvs) != set(metadata):
        missing_summary = sorted(set(csvs) - set(summaries))
        missing_csv = sorted((set(summaries) | set(metadata)) - set(csvs))
        missing_metadata = sorted(set(csvs) - set(metadata))
        raise ResultContractError(
            f"{directory}: unpaired results; missing summaries={missing_summary}, "
            f"missing metadata={missing_metadata}, missing CSVs={missing_csv}")
    if not csvs:
        raise ResultContractError(f"{directory}: no run results")
    return [(csvs[name], summaries[name], metadata[name])
            for name in sorted(csvs)]


def analyze_family(root: Path, family: str, commit: str,
                   allow_dirty: bool) -> dict:
    directory = root / family
    scenario_manifest = json.loads(
        (directory / "scenario_manifest.json").read_text())
    if scenario_manifest["scenario"] != family:
        raise ResultContractError(f"{directory}: scenario manifest mismatch")
    duration = float(scenario_manifest["mission_s"])
    requested_seeds = set(scenario_manifest["seeds"])
    require_yaw = family in {"disturbance", "disturbance_transit"}
    require_target = family == "target_relocation"
    yaw_step_time = scenario_manifest.get("yaw_step_time")
    target_step_time = scenario_manifest.get("target_step_time")

    runs = []
    observed_seeds = set()
    for csv_path, summary_path, metadata_path in result_pairs(directory):
        seed = int(csv_path.stem.removeprefix("run_"))
        metadata = json.loads(metadata_path.read_text())
        if metadata.get("seed") != seed or metadata.get("validation") != "passed":
            raise ResultContractError(f"{metadata_path}: invalid run metadata")
        csv_hash = hashlib.sha256(csv_path.read_bytes()).hexdigest()
        summary_hash = hashlib.sha256(summary_path.read_bytes()).hexdigest()
        if (metadata.get("csv_sha256") != csv_hash or
                metadata.get("summary_sha256") != summary_hash):
            raise ResultContractError(f"{metadata_path}: artifact hash mismatch")
        summary, rows = validate_run(
            csv_path, summary_path, expected_commit=commit,
            expected_seed=seed, expected_scenario=family,
            expected_duration=duration, require_yaw_step=require_yaw,
            require_target_step=require_target, allow_dirty=allow_dirty)
        if require_yaw and abs(float(summary["yaw_step_elapsed"]) -
                               float(yaw_step_time)) > 0.1:
            raise ResultContractError("yaw step timing does not match manifest")
        if require_target and abs(float(summary["target_step_elapsed"]) -
                                  float(target_step_time)) > 0.1:
            raise ResultContractError("target step timing does not match manifest")
        observed_seeds.add(seed)
        runs.append((summary, rows, metadata))
    if observed_seeds != requested_seeds:
        raise ResultContractError(
            f"{directory}: seed set does not match scenario manifest")

    successes = [int(summary["success"]) for summary, _, _ in runs]
    reach = [float(summary["time_to_reach"]) for summary, _, _ in runs
             if float(summary["time_to_reach"]) >= 0.0]
    rmse = [float(summary["station_rmse"]) for summary, _, _ in runs]
    odom_pos = [float(summary["odom_position_rmse"])
                for summary, _, _ in runs]
    odom_yaw = [float(summary["odom_yaw_rmse"])
                for summary, _, _ in runs]
    runtime = [float(metadata["wall_runtime_s"])
               for _, _, metadata in runs]
    n, successes_n = len(runs), sum(successes)
    out = {
        "family": family,
        "n": n,
        "success": successes_n,
        "wilson_lb_95": round(wilson_lb(successes_n, n), 4),
        "reach_s": distribution(reach, 2),
        "station_rmse_m": distribution(rmse, 4),
        "odom_position_rmse_m": distribution(odom_pos, 4),
        "odom_yaw_rmse_rad": distribution(odom_yaw, 5),
        "wall_runtime_s": distribution(runtime, 2),
    }

    if require_yaw or require_target:
        event_applied = 0
        adoption_delay, max_excursion, task_recovery = [], [], []
        yaw_recovery = []
        for summary, rows, _ in runs:
            event_key = "yaw_step_elapsed" if require_yaw else "target_step_elapsed"
            applied_key = "yaw_step_applied" if require_yaw else "target_step_applied"
            event_t = float(summary[event_key])
            if int(summary[applied_key]) != 1 or event_t < 0.0:
                continue
            event_applied += 1
            pre = [row for row in rows if row["t"] < event_t]
            post = [row for row in rows if row["t"] >= event_t]
            if not pre or not post:
                raise ResultContractError("event lacks pre/post observations")
            pre_adoptions = pre[-1]["adoptions"]
            adopted = [row for row in post
                       if row["adoptions"] > pre_adoptions]
            if adopted:
                adoption_delay.append(adopted[0]["t"] - event_t)
            max_excursion.append(max(row["dist"] for row in post))
            recovered = sustained_recovery(
                rows, event_t, lambda row: row["dist"] <= 0.35)
            if recovered is not None:
                task_recovery.append(recovered)
            if require_yaw:
                # true_theta = wrap(true_relay_yaw - odom_yaw_offset) is the
                # ground truth for the gauge-quotient invariant theta_ctrl
                # estimates; true_relay_yaw alone is the raw relay-frame
                # parameter and differs from it by the (unknown, constant)
                # odometry heading offset, so comparing against it directly
                # made this check spuriously unsatisfiable.
                recovered = sustained_recovery(
                    rows, event_t,
                    lambda row: abs(wrap(row["theta_ctrl"] -
                                         row["true_theta"])) <=
                                math.radians(15.0))
                if recovered is not None:
                    yaw_recovery.append(recovered)
        out["event_applied"] = f"{event_applied}/{n}"
        out["adoption_delay_s"] = distribution(adoption_delay, 2)
        out["post_event_max_excursion_m"] = distribution(max_excursion, 3)
        out["task_recovery_s"] = {
            "distribution": distribution(task_recovery, 2),
            "recovered": f"{len(task_recovery)}/{n}",
        }
        if require_yaw:
            out["yaw_recovery_s"] = {
                "distribution": distribution(yaw_recovery, 2),
                "recovered": f"{len(yaw_recovery)}/{n}",
            }
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("campaign_root", type=Path)
    parser.add_argument("--allow-dirty", action="store_true")
    args = parser.parse_args()
    manifest_path = args.campaign_root / "campaign_manifest.json"
    if not manifest_path.is_file():
        raise SystemExit("campaign_manifest.json is required")
    manifest = json.loads(manifest_path.read_text())
    commit = manifest.get("source_commit", "")
    config_path = args.campaign_root / manifest.get("config_file", "")
    if not config_path.is_file():
        raise SystemExit("campaign configuration copy is missing")
    config_hash = hashlib.sha256(config_path.read_bytes()).hexdigest()
    if config_hash != manifest.get("config_sha256"):
        raise SystemExit("campaign configuration hash does not match manifest")
    if manifest.get("campaign_class") != "citable" and not args.allow_dirty:
        raise SystemExit("development campaign requires --allow-dirty")

    try:
        report = [analyze_family(args.campaign_root, family, commit,
                                 args.allow_dirty)
                  for family in FAMILIES if (args.campaign_root / family).is_dir()]
    except (OSError, KeyError, ValueError, ResultContractError) as exc:
        raise SystemExit(f"aggregation refused: {exc}") from exc
    if not report:
        raise SystemExit("no recognized scenario directories")
    payload = {"campaign": manifest, "families": report}
    print(json.dumps(payload, indent=2))
    output = args.campaign_root / "aggregate.json"
    temporary = output.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n")
    temporary.replace(output)
    print(f"\nwritten: {output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

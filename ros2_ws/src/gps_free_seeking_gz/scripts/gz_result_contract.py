#!/usr/bin/env python3
"""Strict validation contract for one Gazebo run result pair."""

from __future__ import annotations

import csv
import math
import re
from pathlib import Path


COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
REQUIRED_COLUMNS = {
    "t", "dist", "mode", "theta_ctrl", "theta_var", "e_hat_norm",
    "retriggers", "adoptions", "odom_pos_err", "odom_yaw_err",
    "relay_sampled", "relay_delivered", "relay_dropped", "relay_outliers",
    "yaw_step_applied", "yaw_step_elapsed", "true_relay_yaw", "true_theta",
    "target_step_applied", "target_step_elapsed", "true_target_x",
    "true_target_y",
}


class ResultContractError(ValueError):
    pass


def parse_summary(path: str | Path) -> dict[str, str]:
    path = Path(path)
    lines = [line.strip() for line in path.read_text().splitlines()
             if line.strip()]
    if len(lines) != 1:
        raise ResultContractError(
            f"{path}: expected exactly one summary line, found {len(lines)}")
    result: dict[str, str] = {}
    for token in lines[0].split():
        if "=" not in token:
            raise ResultContractError(f"{path}: malformed token {token!r}")
        key, value = token.split("=", 1)
        if key in result:
            raise ResultContractError(f"{path}: duplicate key {key}")
        result[key] = value
    return result


def load_csv(path: str | Path) -> list[dict[str, float]]:
    path = Path(path)
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or [])
        missing = REQUIRED_COLUMNS - columns
        if missing:
            raise ResultContractError(
                f"{path}: missing columns {sorted(missing)}")
        rows = []
        for line_number, raw in enumerate(reader, start=2):
            try:
                row = {key: float(value) for key, value in raw.items()}
            except (TypeError, ValueError) as exc:
                raise ResultContractError(
                    f"{path}:{line_number}: non-numeric value") from exc
            if not all(math.isfinite(value) for value in row.values()):
                raise ResultContractError(
                    f"{path}:{line_number}: non-finite value")
            rows.append(row)
    return rows


def _number(summary: dict[str, str], key: str, cast=float):
    if key not in summary:
        raise ResultContractError(f"summary missing {key}")
    try:
        value = cast(summary[key])
    except ValueError as exc:
        raise ResultContractError(f"summary has invalid {key}") from exc
    if cast is float and not math.isfinite(value):
        raise ResultContractError(f"summary has non-finite {key}")
    return value


def validate_run(
    csv_path: str | Path,
    summary_path: str | Path,
    *,
    expected_commit: str | None = None,
    expected_seed: int | None = None,
    expected_scenario: str | None = None,
    expected_duration: float | None = None,
    require_yaw_step: bool = False,
    require_target_step: bool = False,
    expected_yaw_step_time: float | None = None,
    expected_target_step_time: float | None = None,
    allow_dirty: bool = False,
) -> tuple[dict[str, str], list[dict[str, float]]]:
    csv_path = Path(csv_path)
    summary_path = Path(summary_path)
    incomplete = Path(str(csv_path) + ".incomplete")
    if incomplete.exists():
        raise ResultContractError(f"{csv_path}: incomplete marker exists")
    if not csv_path.is_file() or not summary_path.is_file():
        raise ResultContractError("CSV/summary result pair is incomplete")

    summary = parse_summary(summary_path)
    if _number(summary, "complete", int) != 1:
        raise ResultContractError("summary is not complete")
    commit = summary.get("build_commit", "")
    if not COMMIT_RE.fullmatch(commit):
        raise ResultContractError(f"invalid build commit {commit!r}")
    if expected_commit is not None and commit != expected_commit:
        raise ResultContractError(
            f"build commit {commit} does not match {expected_commit}")
    if not allow_dirty and _number(summary, "build_dirty", int) != 0:
        raise ResultContractError("result was produced by a dirty build")
    if expected_seed is not None and _number(summary, "seed", int) != expected_seed:
        raise ResultContractError("summary seed does not match requested seed")
    if expected_scenario is not None and summary.get("scenario") != expected_scenario:
        raise ResultContractError("summary scenario does not match request")

    duration = _number(summary, "expected_duration")
    elapsed = _number(summary, "elapsed")
    if expected_duration is not None and abs(duration - expected_duration) > 1e-6:
        raise ResultContractError("summary duration does not match request")
    if elapsed + 0.05 < duration:
        raise ResultContractError("run ended before its requested duration")

    for key in ("station_rmse", "odom_position_rmse", "odom_yaw_rmse"):
        if _number(summary, key) < 0.0:
            raise ResultContractError(f"summary metric {key} is invalid")
    for key in ("relay_sampled", "relay_delivered", "relay_dropped",
                "relay_outliers"):
        if _number(summary, key, int) < 0:
            raise ResultContractError(f"summary counter {key} is invalid")
    if _number(summary, "relay_sampled", int) == 0:
        raise ResultContractError("no relay samples were generated")
    if require_yaw_step and _number(summary, "yaw_step_applied", int) != 1:
        raise ResultContractError("requested yaw step was not applied")
    if require_target_step and _number(summary, "target_step_applied", int) != 1:
        raise ResultContractError("requested target step was not applied")
    if expected_yaw_step_time is not None and abs(
            _number(summary, "yaw_step_elapsed") -
            expected_yaw_step_time) > 0.1:
        raise ResultContractError("yaw step was applied at the wrong time")
    if expected_target_step_time is not None and abs(
            _number(summary, "target_step_elapsed") -
            expected_target_step_time) > 0.1:
        raise ResultContractError("target step was applied at the wrong time")

    rows = load_csv(csv_path)
    if len(rows) != _number(summary, "csv_rows", int):
        raise ResultContractError("CSV row count does not match summary")
    minimum_rows = max(1, math.floor(4.5 * duration))
    if len(rows) < minimum_rows:
        raise ResultContractError(
            f"CSV is too short: {len(rows)} rows, expected at least {minimum_rows}")
    if any(b["t"] <= a["t"] for a, b in zip(rows, rows[1:])):
        raise ResultContractError("CSV timestamps are not strictly increasing")
    if rows[-1]["t"] + 0.25 < duration:
        raise ResultContractError("CSV does not cover the requested duration")
    if require_yaw_step and not any(row["yaw_step_applied"] >= 0.5 for row in rows):
        raise ResultContractError("CSV contains no applied yaw-step event")
    if require_target_step and not any(
            row["target_step_applied"] >= 0.5 for row in rows):
        raise ResultContractError("CSV contains no applied target-step event")
    return summary, rows

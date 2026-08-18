#!/usr/bin/env python3
"""Validate one Gazebo CSV/summary pair before admitting it to a batch."""

import argparse
from pathlib import Path

from gz_result_contract import ResultContractError, validate_run


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("summary", type=Path)
    parser.add_argument("--commit")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--scenario")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--require-yaw-step", action="store_true")
    parser.add_argument("--require-target-step", action="store_true")
    parser.add_argument("--yaw-step-time", type=float)
    parser.add_argument("--target-step-time", type=float)
    parser.add_argument("--allow-dirty", action="store_true")
    args = parser.parse_args()
    try:
        summary, rows = validate_run(
            args.csv, args.summary, expected_commit=args.commit,
            expected_seed=args.seed, expected_scenario=args.scenario,
            expected_duration=args.duration,
            require_yaw_step=args.require_yaw_step,
            require_target_step=args.require_target_step,
            expected_yaw_step_time=args.yaw_step_time,
            expected_target_step_time=args.target_step_time,
            allow_dirty=args.allow_dirty)
    except (OSError, ResultContractError) as exc:
        print(f"INVALID: {exc}")
        return 2
    print(
        "VALID: "
        f"scenario={summary['scenario']} seed={summary['seed']} "
        f"rows={len(rows)} success={summary['success']} "
        f"rmse={summary['station_rmse']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

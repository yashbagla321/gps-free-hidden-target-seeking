import csv
import importlib.util
import hashlib
import json
import sys
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from gz_result_contract import REQUIRED_COLUMNS  # noqa: E402

AGGREGATOR = PACKAGE.parents[2] / "scripts" / "aggregate_gz.py"
spec = importlib.util.spec_from_file_location("aggregate_gz", AGGREGATOR)
aggregate_gz = importlib.util.module_from_spec(spec)
spec.loader.exec_module(aggregate_gz)


def test_disturbance_uses_logged_event_and_full_recovery_horizon(tmp_path):
    family = tmp_path / "disturbance"
    family.mkdir()
    (family / "scenario_manifest.json").write_text(json.dumps({
        "scenario": "disturbance",
        "mission_s": 15.0,
        "seeds": [1001],
        "yaw_step_time": 2.0,
        "target_step_time": -1.0,
    }))
    fields = sorted(REQUIRED_COLUMNS)
    csv_path = family / "run_1001.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index in range(75):
            t = 0.2 * (index + 1)
            event = t >= 2.0
            adopted = t >= 3.0
            true_yaw = 2.2 + (1.0471975512 if event else 0.0)
            row = {field: 0.0 for field in fields}
            row.update({
                "t": t,
                "dist": 0.2,
                "mode": 2,
                "theta_ctrl": true_yaw if adopted else 2.2,
                "theta_var": 0.001,
                "odom_pos_err": 0.01,
                "odom_yaw_err": 0.001,
                "relay_sampled": index + 1,
                "relay_delivered": index + 1,
                "adoptions": 1 if adopted else 0,
                "yaw_step_applied": 1 if event else 0,
                "yaw_step_elapsed": 2.0 if event else -1.0,
                "true_relay_yaw": true_yaw,
                # This fixture models zero odometry-heading offset (gamma=0),
                # so true_theta = wrap(true_relay_yaw - gamma) = true_yaw.
                "true_theta": true_yaw,
                "target_step_elapsed": -1.0,
                "true_target_x": 1.5,
                "true_target_y": 9.0,
            })
            writer.writerow(row)
    summary = csv_path.with_suffix(".csv.summary")
    summary.write_text(
        f"complete=1 scenario=disturbance seed=1001 "
        f"build_commit={'b' * 40} build_dirty=0 "
        "expected_duration=15 elapsed=15 csv_rows=75 success=1 "
        "time_to_reach=1 station_rmse=0.2 odom_position_rmse=0.01 "
        "odom_yaw_rmse=0.001 relay_sampled=300 relay_delivered=300 "
        "relay_dropped=0 relay_outliers=0 yaw_step_applied=1 "
        "yaw_step_elapsed=2 target_step_applied=0 target_step_elapsed=-1\n")
    (family / "run_1001.meta.json").write_text(json.dumps({
        "seed": 1001,
        "validation": "passed",
        "wall_runtime_s": 15.5,
        "csv_sha256": hashlib.sha256(csv_path.read_bytes()).hexdigest(),
        "summary_sha256": hashlib.sha256(summary.read_bytes()).hexdigest(),
    }))

    result = aggregate_gz.analyze_family(
        tmp_path, "disturbance", "b" * 40, allow_dirty=False)
    assert result["event_applied"] == "1/1"
    assert result["adoption_delay_s"]["median"] == 1.0
    assert result["yaw_recovery_s"]["recovered"] == "1/1"
    assert result["task_recovery_s"]["recovered"] == "1/1"

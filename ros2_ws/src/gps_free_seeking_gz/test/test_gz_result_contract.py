import csv
import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from gz_result_contract import (  # noqa: E402
    REQUIRED_COLUMNS,
    ResultContractError,
    validate_run,
)


def make_result(tmp_path, *, rows=25, summary_rows=25, complete=1,
                dirty=0, yaw_step=0):
    csv_path = tmp_path / "run_1001.csv"
    fields = sorted(REQUIRED_COLUMNS)
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index in range(rows):
            t = 0.2 * (index + 1)
            row = {field: 0.0 for field in fields}
            row.update({
                "t": t,
                "dist": 0.1,
                "mode": 2,
                "theta_ctrl": 2.2,
                "theta_var": 0.001,
                "odom_pos_err": 0.01,
                "odom_yaw_err": 0.001,
                "relay_sampled": index + 1,
                "relay_delivered": index + 1,
                "yaw_step_applied": yaw_step,
                "yaw_step_elapsed": 2.0 if yaw_step else -1.0,
                "true_relay_yaw": 2.2,
                "target_step_elapsed": -1.0,
                "true_target_x": 1.5,
                "true_target_y": 9.0,
            })
            writer.writerow(row)
    summary = csv_path.with_suffix(".csv.summary")
    summary.write_text(
        f"complete={complete} scenario=nominal seed=1001 "
        f"build_commit={'a' * 40} build_dirty={dirty} "
        f"expected_duration=5.0 elapsed=5.0 csv_rows={summary_rows} "
        "success=1 time_to_reach=1.0 station_rmse=0.1 "
        "odom_position_rmse=0.01 odom_yaw_rmse=0.001 "
        "relay_sampled=100 relay_delivered=100 relay_dropped=0 "
        "relay_outliers=0 "
        f"yaw_step_applied={yaw_step} "
        f"yaw_step_elapsed={2.0 if yaw_step else -1.0} "
        "target_step_applied=0 target_step_elapsed=-1.0\n")
    return csv_path, summary


def test_accepts_complete_provenance_matched_pair(tmp_path):
    csv_path, summary = make_result(tmp_path)
    parsed, rows = validate_run(
        csv_path, summary, expected_commit="a" * 40,
        expected_seed=1001, expected_scenario="nominal",
        expected_duration=5.0)
    assert parsed["complete"] == "1"
    assert len(rows) == 25


def test_rejects_appended_summary(tmp_path):
    csv_path, summary = make_result(tmp_path)
    with summary.open("a") as stream:
        stream.write(summary.read_text())
    with pytest.raises(ResultContractError, match="exactly one summary line"):
        validate_run(csv_path, summary)


@pytest.mark.parametrize(
    "kwargs,match",
    [
        ({"summary_rows": 24}, "row count"),
        ({"complete": 0}, "not complete"),
        ({"dirty": 1}, "dirty build"),
    ],
)
def test_rejects_invalid_result_contract(tmp_path, kwargs, match):
    csv_path, summary = make_result(tmp_path, **kwargs)
    with pytest.raises(ResultContractError, match=match):
        validate_run(csv_path, summary)


def test_requires_logged_disturbance_event(tmp_path):
    csv_path, summary = make_result(tmp_path, yaw_step=0)
    with pytest.raises(ResultContractError, match="yaw step was not applied"):
        validate_run(csv_path, summary, require_yaw_step=True)

    csv_path, summary = make_result(tmp_path, yaw_step=1)
    validate_run(csv_path, summary, require_yaw_step=True)

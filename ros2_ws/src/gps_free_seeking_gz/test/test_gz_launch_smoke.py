import os
import signal
import subprocess
import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from gz_result_contract import validate_run  # noqa: E402


def gz_pids():
    result = subprocess.run(
        ["pgrep", "-f", "^gz sim"], text=True, capture_output=True,
        check=False)
    return {int(value) for value in result.stdout.split() if value.isdigit()}


@pytest.mark.skipif(os.name != "posix", reason="Gazebo smoke test requires Linux")
def test_headless_launch_writes_one_complete_result(tmp_path):
    before = gz_pids()
    if before:
        pytest.skip("a Gazebo server is already running")
    command = [
        "ros2", "launch", "gps_free_seeking_gz",
        "gfs_seeking_gz.launch.py", "seed:=7001", "scenario:=smoke",
        "mission_s:=3", "station_window_s:=1",
        f"output_dir:={tmp_path}", "output_name:=run_7001.csv",
    ]
    process = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, start_new_session=True)
    try:
        output, _ = process.communicate(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate()
        pytest.fail(f"Gazebo smoke test timed out:\n{output}")
    finally:
        for pid in gz_pids() - before:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    assert process.returncode == 0, output
    csv_path = tmp_path / "run_7001.csv"
    summary_path = tmp_path / "run_7001.csv.summary"
    assert not (tmp_path / "run_7001.csv.incomplete").exists()
    validate_run(
        csv_path, summary_path, expected_seed=7001,
        expected_scenario="smoke", expected_duration=3.0,
        allow_dirty=True)

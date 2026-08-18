#!/usr/bin/env bash
# Provenance-locked Gazebo batch runner. A run enters the campaign only after
# its CSV/summary pair passes the strict result contract.
set -Eeuo pipefail

usage() {
  echo "usage: run_gz_batch.sh SCENARIO N [CAMPAIGN_ID]" >&2
  echo "scenarios: nominal stress disturbance disturbance_transit target_relocation" >&2
  exit 2
}

SCENARIO="${1:-}"
N="${2:-}"
CAMPAIGN_ID="${3:-$(date -u +%Y%m%dT%H%M%SZ)}"
[[ -n "$SCENARIO" && "$N" =~ ^[1-9][0-9]*$ ]] || usage
[[ "$CAMPAIGN_ID" =~ ^[A-Za-z0-9._-]+$ ]] || {
  echo "invalid campaign id: $CAMPAIGN_ID" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_SHARE="$(cd "$SCRIPT_DIR/.." && pwd)"
PROVENANCE="$PKG_SHARE/config/build_provenance.env"
[[ -f "$PROVENANCE" ]] || {
  echo "missing build provenance: rebuild gps_free_seeking_gz" >&2
  exit 2
}
# shellcheck disable=SC1090
source "$PROVENANCE"
[[ "$GFS_BUILD_COMMIT" =~ ^[0-9a-f]{40}$ ]] || {
  echo "installed package has invalid build commit" >&2
  exit 2
}

REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)" || {
  echo "run from a CooperativeLocalization Git checkout" >&2
  exit 2
}
SOURCE_COMMIT="$(git -C "$REPO_ROOT" rev-parse HEAD)"
[[ "$SOURCE_COMMIT" == "$GFS_BUILD_COMMIT" ]] || {
  echo "source/build mismatch: source=$SOURCE_COMMIT build=$GFS_BUILD_COMMIT" >&2
  echo "rebuild the ROS workspace from this checkout" >&2
  exit 2
}

RELEVANT_STATUS="$(git -C "$REPO_ROOT" status --porcelain --untracked-files=all -- \
  gps_free_seeking/CMakeLists.txt gps_free_seeking/include \
  gps_free_seeking/src gps_free_seeking/config gps_free_seeking/scripts \
  gps_free_seeking/ros2_ws/src)"
ALLOW_DIRTY="${GFS_ALLOW_DIRTY:-0}"
if [[ "$GFS_BUILD_DIRTY" != "0" || -n "$RELEVANT_STATUS" ]]; then
  if [[ "$ALLOW_DIRTY" != "1" ]]; then
    echo "refusing a citable campaign from dirty source/build state" >&2
    echo "$RELEVANT_STATUS" >&2
    echo "commit the validation changes, rebuild, then rerun" >&2
    exit 2
  fi
  CAMPAIGN_CLASS="development"
else
  CAMPAIGN_CLASS="citable"
fi

MISSION_S="90.0"
MISSION_SECONDS=90
REQUIRE_YAW=0
REQUIRE_TARGET=0
YAW_STEP_TIME="-1.0"
TARGET_STEP_TIME="-1.0"
EXTRA_ARGS=()
case "$SCENARIO" in
  nominal) ;;
  stress)
    EXTRA_ARGS+=(dropout:=0.3 delay_s:=0.2 delay_jitter_s:=0.1
                 outlier_prob:=0.05) ;;
  disturbance)
    MISSION_S="120.0"; MISSION_SECONDS=120; REQUIRE_YAW=1
    YAW_STEP_TIME="20.0"
    EXTRA_ARGS+=(yaw_step_time:=20.0 yaw_step_deg:=60.0) ;;
  disturbance_transit)
    MISSION_S="120.0"; MISSION_SECONDS=120; REQUIRE_YAW=1
    YAW_STEP_TIME="6.5"
    EXTRA_ARGS+=(yaw_step_time:=6.5 yaw_step_deg:=60.0) ;;
  target_relocation)
    MISSION_S="120.0"; MISSION_SECONDS=120; REQUIRE_TARGET=1
    TARGET_STEP_TIME="20.0"
    EXTRA_ARGS+=(target_step_time:=20.0 target_step_dx:=2.0
                 target_step_dy:=0.0) ;;
  *) usage ;;
esac

related_ros_nodes() {
  ros2 node list 2>/dev/null | grep -E \
    '^/(gfs_bridge|gfs_seeker_node|gz_evaluator_node|relay_emulator_node)$' || true
}

ensure_no_related_nodes() {
  local nodes=""
  for _ in 1 2 3 4 5; do
    nodes="$(related_ros_nodes)"
    [[ -z "$nodes" ]] && return 0
    sleep 1
  done
  echo "related ROS nodes are still present:" >&2
  echo "$nodes" >&2
  return 1
}

ensure_no_related_nodes
if pgrep -f '^gz sim' >/dev/null; then
  echo "refusing to start while a Gazebo server is already running" >&2
  pgrep -af '^gz sim' >&2
  exit 2
fi

RESULTS_ROOT="${GFS_RESULTS_ROOT:-$REPO_ROOT/gps_free_seeking/results/campaign2027/ros_gz}"
CAMPAIGN_ROOT="$RESULTS_ROOT/$CAMPAIGN_CLASS/$SOURCE_COMMIT/$CAMPAIGN_ID"
OUT="$CAMPAIGN_ROOT/$SCENARIO"
[[ ! -e "$OUT" ]] || {
  echo "refusing to overwrite scenario output: $OUT" >&2
  exit 2
}
mkdir -p "$OUT"

CONFIG_COPY="$CAMPAIGN_ROOT/gfs_seeking_gz.yaml"
if [[ ! -f "$CONFIG_COPY" ]]; then
  cp "$PKG_SHARE/config/gfs_seeking_gz.yaml" "$CONFIG_COPY"
fi
CONFIG_SHA256="$(sha256sum "$CONFIG_COPY" | awk '{print $1}')"
MANIFEST="$CAMPAIGN_ROOT/campaign_manifest.json"
if [[ ! -f "$MANIFEST" ]]; then
  GZ_VERSION="$(gz sim --versions 2>/dev/null | head -1)"
  ROS_GZ_SIM_VERSION="$(ros2 pkg xml ros_gz_sim 2>/dev/null | \
    sed -n 's:.*<version>\(.*\)</version>.*:\1:p' | head -1)"
  python3 - "$MANIFEST" "$SOURCE_COMMIT" "$GFS_BUILD_DIRTY" \
    "$CAMPAIGN_CLASS" "$CONFIG_SHA256" "${ROS_DISTRO:-unknown}" \
    "$GZ_VERSION" "$ROS_GZ_SIM_VERSION" <<'PY'
import json
import pathlib
import sys
import time

path, commit, dirty, campaign_class, config_hash, ros_distro, gz_version, \
    ros_gz_sim_version = sys.argv[1:]
data = {
    "schema": 1,
    "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "source_commit": commit,
    "build_commit": commit,
    "build_dirty": int(dirty),
    "campaign_class": campaign_class,
    "config_file": "gfs_seeking_gz.yaml",
    "config_sha256": config_hash,
    "ros_distro": ros_distro,
    "gazebo_version": gz_version,
    "ros_gz_sim_version": ros_gz_sim_version,
}
pathlib.Path(path).write_text(json.dumps(data, indent=2) + "\n")
PY
else
  python3 - "$MANIFEST" "$SOURCE_COMMIT" "$CONFIG_SHA256" <<'PY'
import json
import sys

data = json.load(open(sys.argv[1]))
if data["source_commit"] != sys.argv[2] or data["config_sha256"] != sys.argv[3]:
    raise SystemExit("campaign manifest does not match source/config")
PY
fi

python3 - "$OUT/scenario_manifest.json" "$SCENARIO" "$N" "$MISSION_S" \
  "$SOURCE_COMMIT" "$CONFIG_SHA256" "$YAW_STEP_TIME" \
  "$TARGET_STEP_TIME" "${EXTRA_ARGS[*]}" <<'PY'
import json
import pathlib
import sys

path, scenario, trials, mission, commit, config_hash, yaw_step_time, \
    target_step_time, overrides = sys.argv[1:]
data = {
    "schema": 1,
    "scenario": scenario,
    "trials": int(trials),
    "seeds": list(range(1001, 1001 + int(trials))),
    "mission_s": float(mission),
    "source_commit": commit,
    "config_sha256": config_hash,
    "yaw_step_time": float(yaw_step_time),
    "target_step_time": float(target_step_time),
    "launch_overrides": overrides.split(),
}
pathlib.Path(path).write_text(json.dumps(data, indent=2) + "\n")
PY

VALIDATOR=(python3 "$SCRIPT_DIR/validate_gz_run.py")
[[ "$ALLOW_DIRTY" == "1" ]] && VALIDATOR+=(--allow-dirty)
[[ "$REQUIRE_YAW" == "1" ]] && VALIDATOR+=(--require-yaw-step \
  --yaw-step-time "$YAW_STEP_TIME")
[[ "$REQUIRE_TARGET" == "1" ]] && VALIDATOR+=(--require-target-step \
  --target-step-time "$TARGET_STEP_TIME")
RUN_TIMEOUT="${GFS_RUN_TIMEOUT_S:-$((2 * MISSION_SECONDS + 60))}"

# Kills leaked Gazebo servers AND leaked ROS nodes. On WSL, ros_gz's
# parameter_bridge (node name /gfs_bridge) exhibits the same "does not
# terminate on SIGINT within the launch shutdown grace period" behavior we
# already observed and fixed for the gz-sim server itself; without an
# explicit kill it can outlive ensure_no_related_nodes's 5s retry window,
# especially after many launches in one long session. Kill all five leaked
# process families here so ensure_no_related_nodes always finds a clean
# graph.
#
# 'gz sim' and 'parameter_bridge' are generic names other, unrelated
# Gazebo/ROS sessions on the same machine could also be running, so these
# two kills are scoped to THIS script's own process group (-g), not the
# whole machine. The three gfs_* executables are package-specific and safe
# to match by name alone.
SCRIPT_PGID="$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ')"
cleanup_gazebo() {
  local leaked
  if [[ -n "$SCRIPT_PGID" ]]; then
    leaked="$(pgrep -g "$SCRIPT_PGID" -f '^gz sim' || true)"
    if [[ -n "$leaked" ]]; then
      echo "cleaning leaked Gazebo server(s): $leaked" >&2
      pkill -9 -g "$SCRIPT_PGID" -f '^gz sim' || true
    fi
    leaked="$(pgrep -g "$SCRIPT_PGID" -f 'parameter_bridge' || true)"
    if [[ -n "$leaked" ]]; then
      echo "cleaning leaked bridge process(es): $leaked" >&2
      pkill -9 -g "$SCRIPT_PGID" -f 'parameter_bridge' || true
    fi
  fi
  leaked="$(pgrep -f 'gfs_seeker_node|gz_evaluator_node|relay_emulator_node' || true)"
  if [[ -n "$leaked" ]]; then
    echo "cleaning leaked ROS node process(es): $leaked" >&2
    pkill -9 -f 'gfs_seeker_node|gz_evaluator_node|relay_emulator_node' || true
  fi
  sleep 1
}
trap cleanup_gazebo EXIT INT TERM

for i in $(seq 1 "$N"); do
  SEED=$((1000 + i))
  CSV="$OUT/run_${SEED}.csv"
  SUMMARY="$CSV.summary"
  LOG="$OUT/run_${SEED}.log"
  META="$OUT/run_${SEED}.meta.json"
  echo "[$i/$N] $SCENARIO seed=$SEED"
  STARTED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  START_NS="$(date +%s%N)"
  set +e
  timeout --signal=INT --kill-after=15s "${RUN_TIMEOUT}s" \
    ros2 launch gps_free_seeking_gz gfs_seeking_gz.launch.py \
      config:="$CONFIG_COPY" seed:="$SEED" scenario:="$SCENARIO" \
      mission_s:="$MISSION_S" output_dir:="$OUT" \
      output_name:="run_${SEED}.csv" fail_if_output_exists:=true \
      "${EXTRA_ARGS[@]}" >"$LOG" 2>&1
  RC=$?
  END_NS="$(date +%s%N)"
  set -e
  cleanup_gazebo
  ensure_no_related_nodes
  if [[ "$RC" -ne 0 ]]; then
    echo "launch failed for seed $SEED (rc=$RC); see $LOG" >&2
    exit "$RC"
  fi
  if grep -Eq '\[ERROR\] \[(relay_emulator_node|gfs_seeker_node|gz_evaluator_node|parameter_bridge)-[0-9]+\]: process has died' "$LOG"; then
    echo "required ROS node failed for seed $SEED; see $LOG" >&2
    exit 1
  fi
  "${VALIDATOR[@]}" "$CSV" "$SUMMARY" --commit "$SOURCE_COMMIT" \
    --seed "$SEED" --scenario "$SCENARIO" --duration "$MISSION_S"
  CSV_SHA256="$(sha256sum "$CSV" | awk '{print $1}')"
  SUMMARY_SHA256="$(sha256sum "$SUMMARY" | awk '{print $1}')"
  python3 - "$META" "$SEED" "$STARTED_UTC" "$START_NS" "$END_NS" \
    "$CSV_SHA256" "$SUMMARY_SHA256" "${EXTRA_ARGS[*]}" <<'PY'
import json
import pathlib
import sys

path, seed, started, start_ns, end_ns, csv_hash, summary_hash, overrides = \
    sys.argv[1:]
data = {
    "schema": 1,
    "seed": int(seed),
    "started_utc": started,
    "wall_runtime_s": (int(end_ns) - int(start_ns)) / 1e9,
    "launch_exit_code": 0,
    "validation": "passed",
    "csv_sha256": csv_hash,
    "summary_sha256": summary_hash,
    "launch_overrides": overrides.split(),
}
temporary = pathlib.Path(path + ".tmp")
temporary.write_text(json.dumps(data, indent=2) + "\n")
temporary.replace(path)
PY
done

trap - EXIT INT TERM
cleanup_gazebo
echo "validated batch complete: $OUT"
grep -H '^complete=1 ' "$OUT"/*.summary | sort

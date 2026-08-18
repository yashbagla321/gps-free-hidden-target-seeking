# gps_free_seeking_gz

Gazebo validation for GPS-free hidden-target seeking. This package exists to
demonstrate what the numerical campaign cannot, per the pre-submission
review bar:

1. **Physics-driven odometry.** The vehicle is a differential-drive model
   with real wheel joints; `/odom` comes from the DiffDrive system's wheel
   integration (slip, inertia, contact), not from ground truth.
2. **Asynchronous delivery through real ROS interfaces.** Relay packets are
   stamped at measurement time and delivered by a separate sim-time timer
   with configurable dropout, fixed delay, and jitter.
3. **Relay-frame disturbance through the ROS graph.** Yaw steps and target
   relocation are injected inside the emulator and reach the estimator only
   as changed packet content.

Ground-truth isolation is auditable:
`ros2 node info /gfs_seeker_node` must list only `/model/gfs_vehicle/odometry`
(remapped to `/odom`), `/relay/packet`, and `/clock` as data subscriptions.
Truth (`/gfs/truth_odom`) is consumed only by the emulator and evaluator.

## Build (WSL, ROS 2 Jazzy + modern Gazebo)

    cd gps_free_seeking/ros2_ws
    colcon build --packages-select gps_free_seeking_msgs gps_free_seeking_gz
    source install/setup.bash

## Run

Single seeded run:

    ros2 launch gps_free_seeking_gz gfs_seeking_gz.launch.py \
      seed:=12 output_dir:=/tmp/gfs_smoke output_name:=run_12.csv

The launch layer assigns explicit ROS parameter types, so both `mission_s:=90`
and `mission_s:=90.0` are accepted. Existing output is never overwritten.

For citable batches, commit the relevant source and rebuild first. The runner
requires the installed binary's embedded commit to equal the source checkout,
rejects dirty source/build state, creates an immutable campaign directory, and
validates every result before proceeding:

    CAMPAIGN=gazebo_validation_01
    RUNNER=./install/gps_free_seeking_gz/share/gps_free_seeking_gz/scripts/run_gz_batch.sh
    "$RUNNER" nominal 30 "$CAMPAIGN"
    "$RUNNER" stress 30 "$CAMPAIGN"
    "$RUNNER" disturbance 30 "$CAMPAIGN"
    "$RUNNER" disturbance_transit 30 "$CAMPAIGN"
    "$RUNNER" target_relocation 30 "$CAMPAIGN"

Results live under
`results/campaign2027/ros_gz/citable/<40-character-commit>/<campaign>/`. Every run
has one CSV, one atomic single-line summary, one launch log, and one metadata
file containing wall runtime and artifact hashes. Campaign and
scenario manifests record the commit, configuration hash, overrides, seeds,
ROS distribution, and requested duration. Interrupted runs write an
`.incomplete` marker and cannot be aggregated.

`GFS_ALLOW_DIRTY=1` permits development smoke batches, but stores them under
`development/` and they are rejected by the citable aggregator by default.

Aggregate a completed campaign from the repository root:

    python3 gps_free_seeking/scripts/aggregate_gz.py \
      gps_free_seeking/results/campaign2027/ros_gz/citable/<commit>/<campaign>

## Consistency check

The seeker links the same header-only core as the offline campaign. Every
supervisor and registration threshold is explicit in the Gazebo YAML. The
odometry uncertainty is expressed as a continuous-time random-walk intensity,
so changing the Gazebo publication rate does not silently change the
certificate. The evaluator separately logs wheel-odometry position/yaw error
against truth for empirical calibration.

The `/relay/diagnostics` topic is validation-only and records sampled,
delivered, dropped, and outlier counts plus the actual yaw/target event times.
The online seeker does not subscribe to it. Aggregation reports disturbance
application, estimator adoption, yaw recovery, and task recovery separately.

Run the package tests after every build:

    colcon test --packages-select gps_free_seeking_gz
    colcon test-result --verbose

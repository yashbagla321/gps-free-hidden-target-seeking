# Gazebo Validation Checklist

Use this gate before starting a campaign whose results may be reported.

- [ ] Relevant source is committed and the checkout is on the intended commit.
- [ ] `colcon build` was run after the commit.
- [ ] `colcon test-result --verbose` reports no failures and includes the
      result-contract and headless-launch tests.
- [ ] `gz sdf -k` reports that `gfs_hidden_target.sdf` is valid.
- [ ] No `gz sim` server is running before the batch starts.
- [ ] `ros2 node info /gfs_seeker_node` lists wheel odometry and relay packets,
      but not `/gfs/truth_odom` or `/relay/diagnostics`.
- [ ] The batch runner reports `VALID` for every seed.
- [ ] Every scenario has the requested seed set and no `.incomplete` files.
- [ ] Every summary has `complete=1`, one line, the expected 40-character
      commit, `build_dirty=0`, and nonnegative RMSE values.
- [ ] Disturbance scenarios report `event_applied=N/N`; detection, adoption,
      yaw recovery, and task recovery are interpreted as separate outcomes.
- [ ] Odometry position/yaw RMSE is inspected before accepting certificate
      statistics; assumed random-walk intensity is revised if miscalibrated.
- [ ] Aggregation completes without `--allow-dirty` and writes `aggregate.json`
      under the immutable campaign directory.

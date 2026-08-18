# Clean Campaign Summary

Source commit: `212a98b29759e3e4f21e29c33341a47c3da9c80c`

Every manifest reports the same compiled/source commit, `git_dirty=false`, and
the same run ID. The final checksum ledger covers all files in this directory.

## Certificate Calibration

- Exact-pose conditioning: 20,000 trials; pooled 90/95/99% coverage is
  0.9115/0.9576/0.9909.
- Correlated translation-odometry coverage: 20,000 trials; pooled 90/95/99%
  coverage is 0.8993/0.9478/0.9894.
- Across odometry cells, empirical/predicted variance ratios are 0.88-1.07 and
  cellwise 95% coverage is 0.940-0.956.

## Estimator Baselines

All rows use 200 paired randomized geometries.

| Method | Success | Median RMSE (m) | 95% bootstrap CI (m) | Median TTG (s) | Runtime (us/update) |
|---|---:|---:|---:|---:|---:|
| Proposed | 200/200 | 0.0639 | [0.0583, 0.0694] | 14.00 | 3.60 |
| Proposed + smoother | 200/200 | 0.0639 | [0.0583, 0.0694] | 14.00 | 181.72 |
| Known-yaw oracle | 200/200 | 0.0653 | [0.0594, 0.0715] | 12.85 | 4.06 |
| Sequential EKF | 200/200 | 0.0868 | [0.0797, 0.0950] | 14.17 | 0.28 |
| Endpoint-only | 194/200 | 0.0708 | [0.0641, 0.0785] | 27.06 | 0.25 |
| Fixed-decay excitation | 200/200 | 0.0625 | [0.0588, 0.0682] | 14.01 | 3.49 |
| No excitation | 0/200 | 10.3256 | [9.7793, 11.0267] | -- | 3.62 |

The paired median EKF-minus-proposed RMSE difference is +0.0193 m, with a
10,000-replicate percentile-bootstrap 95% interval [+0.0146,+0.0274] m.

## Robustness

- Relay noise: 200/200 successes in every cell through 0.4 m range noise and
  4 deg bearing noise; the hardest bearing cell has 0.170 m median RMSE.
- Communication: 200/200 successes in every cell through 50% dropout, 0.5 s
  fixed delay, 0.2 s jitter, and 20% packet outliers. Median RMSE at 20%
  outliers is 0.204 m.
- Long-horizon drift: 100/100 successes per cell over 600 s. Median station
  RMSE is 0.058-0.064 m while median dead-reckoning error reaches 27.2 m at
  5 cm/s body-frame bias.

## Odometry Operating Boundary

- Translation noise per 10 ms increment: 200/200 at 0.5 cm, 188/200 at 1 cm,
  and 0/200 at 2-5 cm because the honest certificate refuses acquisition.
- Bias through 5 cm/s, scale error through 10%, and heading drift through
  0.5 deg/s: 200/200 in every cell.
- The smoother does not extend the white-noise boundary: 60/60 at 0.5 cm,
  59/60 at 1 cm, and 0/60 at 2-5 cm.

## Relay-Frame Disturbances

- Transit and station scenarios retain task success in all 150/150 trials for
  every 20/40/80 deg yaw step and both excitation policies.
- Transit recovery among initially out-of-tolerance trials is approximately
  74-75%, 78-80%, and 100% for 20/40/80 deg changes.
- Station recovery after target relocation is approximately 10-11%, 67%, and
  98% for 20/40/80 deg changes. This demonstrates size- and motion-dependent
  calibration recovery while task recovery remains complete.
- No-disturbance controls observed 0/200 adoptions per policy; the 95% Wilson
  upper bound is 1.88% per policy.

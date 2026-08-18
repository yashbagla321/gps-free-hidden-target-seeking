#pragma once
// Core data types shared by the CLI simulator, tests, and (later) ROS nodes.
// Plan reference: FINAL_IMPLEMENTATION_PLAN.md §4.1.
//
// Frames (plan §3.1):
//   W : inaccessible global frame.
//   O : locally consistent metric odometry frame, unknown SE(2) placement in W.
//   B : unknown relay frame (yaw psi in W); theta = psi - gamma is the
//       relay-to-odometry yaw, the calibration target.

#include <cstdint>

#include "Math.hpp"

namespace gfs {

// Native range-bearing packet from the relay: measurements to the vehicle and
// to the hidden target, expressed in the relay's own frame B.
struct RelayPacket {
    double t = 0.0;            // timestamp [s]
    double r_v = 0.0;          // range relay->vehicle [m]
    double beta_v = 0.0;       // bearing relay->vehicle in B [rad]
    double r_t = 0.0;          // range relay->target [m]
    double beta_t = 0.0;       // bearing relay->target in B [rad]
    double sigma_r = 0.0;      // per-channel noise std devs (native)
    double sigma_beta = 0.0;
    bool valid = false;        // false: dropped / not yet delivered

    Vec2 lv() const { return {r_v * std::cos(beta_v), r_v * std::sin(beta_v)}; }
    Vec2 lt() const { return {r_t * std::cos(beta_t), r_t * std::sin(beta_t)}; }
    // Beacon-frame task difference d = l^t - l^v = R(theta)^T e_O + noise.
    Vec2 d() const { return lt() - lv(); }
};

// Body-frame SE(2) odometry increment between consecutive control steps.
struct OdometrySample {
    double t = 0.0;            // timestamp at end of interval [s]
    Vec2 dxy;                  // body-frame translation increment [m]
    double dtheta = 0.0;       // heading increment [rad]
    double sigma_xy = 0.0;     // increment noise std dev per axis [m]
    double sigma_dtheta = 0.0; // heading increment noise std dev [rad]
};

enum class Mode : std::uint8_t { kExcite = 0, kSeek = 1, kMaintain = 2 };

inline const char* modeName(Mode m) {
    switch (m) {
        case Mode::kExcite: return "EXCITE";
        case Mode::kSeek: return "SEEK";
        case Mode::kMaintain: return "MAINTAIN";
    }
    return "?";
}

// Estimator state published every packet (plan §4.1 EstimatorOutput).
struct EstimatorOutput {
    double theta_hat = 0.0;        // relay-to-odometry yaw estimate [rad]
    double theta_variance = 1e6;   // certificate variance [rad^2]
    Vec2 e_hat;                    // filtered task vector in O [m]
    Vec2 e_raw;                    // instantaneous R(theta_hat) d_k [m]
    double info_theta = 0.0;       // Schur-complement yaw information I_theta
    double spread = 0.0;           // S_v geometry statistic [m^2]
    double correlation = 0.0;      // registration correlation in [0,1]
    int window_packets = 0;        // packets currently in the window
    bool certified = false;        // 1.96*sqrt(var) <= cert threshold
    bool valid = false;            // estimate usable at all (>=2 views)
    bool smoother_active = false;  // fixed-lag refinement currently accepted
    double runtime_us = 0.0;       // last update cost
};

// Final saturated command (unicycle).
struct SeekCommand {
    double v = 0.0;        // linear velocity [m/s]
    double omega = 0.0;    // angular velocity [rad/s]
    Mode mode = Mode::kExcite;
    bool retriggered = false;  // true on SEEK/MAINTAIN -> EXCITE transitions
};

}  // namespace gfs

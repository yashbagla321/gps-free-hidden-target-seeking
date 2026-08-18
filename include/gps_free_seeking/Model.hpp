#pragma once
// LEGACY pre-SE(2)-gauge model, kept for reference only. This vehicle
// carries a compass (world-aligned heading), which conflicts with the
// current paper's premise of body-frame-only odometry and an unknown
// relay-to-odometry yaw (see WorldSE2.hpp, used by every current paper
// result: campaign_main.cpp, icra_main.cpp/smoke_main.cpp, viz_dump.cpp,
// Se2CoreTests.cpp). Only main.cpp and CoreTests.cpp (pre-ICRA targets,
// excluded from the public companion repo's default build) still use this.
//
// World model, sensing model, and simulation parameters for GPS-free
// hidden-target seeking through one unknown-pose range-bearing relay beacon.
//
// Frames and notation (kept consistent with the baseline paper):
//   q    : vehicle position in the world frame W (UNKNOWN to the vehicle here)
//   p    : hidden static target position in W (unknown)
//   x    : relay beacon position in W (unknown)
//   psi  : relay beacon yaw in W (unknown)
//   e    : p - q, the task-relevant relative target vector (control frame)
//   z    : x - q, relative beacon vector
//
// The vehicle carries a compass (heading reference independent of GPS), so its
// odometry increments are expressed in a world-aligned control frame. It has
// no absolute position sensor in the headline scenarios; a noisy GPS can be
// enabled for the anchoring / comparison studies.

#include "Math.hpp"

namespace gfs {

struct NoiseParams {
    double sigma_range = 0.10;         // beacon range noise [m]
    double sigma_bearing = 0.0175;     // beacon bearing noise [rad] (~1 deg)
    double sigma_odo = 0.02;           // odometry velocity noise density [m/sqrt(s)]
    Vec2 odo_bias{0.01, -0.005};       // constant odometry velocity bias [m/s]
    double sigma_gps = 0.0;            // GPS position noise [m]; <=0 disables GPS
};

struct ControlParams {
    double k_p = 0.4;                  // seeking gain
    double k_e = 2.0;                  // relative-target observer gain [1/s]
    double gamma_psi = 6.0;            // yaw adaptation gain
    double v_max = 1.5;                // speed saturation [m/s]
    double exc_amp = 0.8;              // exploratory excitation amplitude [m/s]
    double exc_omega = 1.2;            // excitation frequency [rad/s]
    double exc_lambda = 0.05;          // excitation decay rate [1/s]
    double window = 1.0;               // displacement window for yaw update [s]
    double d_min = 0.20;               // minimum window displacement to adapt [m]
    double disp_reg = 0.02;            // displacement regularizer [m^2]; absorbs
                                       // odometry noise floor in the yaw gradient
};

struct SimParams {
    double dt = 0.005;                 // integration step [s]
    double meas_period = 0.05;         // beacon measurement period [s] (20 Hz)
    double T = 120.0;                  // horizon [s]
    Vec2 q0{0.0, 0.0};                 // true initial vehicle position
    Vec2 target{12.0, 8.0};            // true target position p
    Vec2 beacon{6.0, -4.0};            // true beacon position x
    double beacon_yaw = 2.2;           // true beacon yaw psi [rad]
    NoiseParams noise;
    ControlParams ctrl;
};

// One beacon-frame range-bearing measurement pair (to vehicle and to target).
struct BeaconMeasurement {
    double r_v = 0.0;   // range beacon -> vehicle
    double beta_v = 0.0;  // bearing beacon -> vehicle, in the beacon's own frame
    double r_t = 0.0;   // range beacon -> target
    double beta_t = 0.0;  // bearing beacon -> target, in the beacon's own frame

    // Reconstructed beacon-frame position vectors.
    Vec2 bv() const { return {r_v * std::cos(beta_v), r_v * std::sin(beta_v)}; }
    Vec2 bt() const { return {r_t * std::cos(beta_t), r_t * std::sin(beta_t)}; }
};

class World {
  public:
    World(const SimParams& prm, unsigned seed) : prm_(prm), rng_(seed), q_(prm.q0) {}

    const Vec2& q() const { return q_; }
    const SimParams& prm() const { return prm_; }

    // Advance true vehicle state with commanded velocity u; returns the
    // odometry increment reported to the vehicle (bias + noise corrupted).
    Vec2 step(const Vec2& u) {
        const double dt = prm_.dt;
        q_ += u * dt;
        Vec2 odo = u * dt + prm_.noise.odo_bias * dt;
        const double s = prm_.noise.sigma_odo * std::sqrt(dt);
        odo += Vec2{rng_.gauss(s), rng_.gauss(s)};
        return odo;
    }

    BeaconMeasurement measure() {
        BeaconMeasurement m;
        const Vec2 rel_v = rotateT(prm_.beacon_yaw, q_ - prm_.beacon);
        const Vec2 rel_t = rotateT(prm_.beacon_yaw, prm_.target - prm_.beacon);
        m.r_v = rel_v.norm() + rng_.gauss(prm_.noise.sigma_range);
        m.beta_v = std::atan2(rel_v.y, rel_v.x) + rng_.gauss(prm_.noise.sigma_bearing);
        m.r_t = rel_t.norm() + rng_.gauss(prm_.noise.sigma_range);
        m.beta_t = std::atan2(rel_t.y, rel_t.x) + rng_.gauss(prm_.noise.sigma_bearing);
        return m;
    }

    // Noisy absolute position fix; only meaningful when sigma_gps > 0.
    Vec2 gps() {
        return q_ + Vec2{rng_.gauss(prm_.noise.sigma_gps), rng_.gauss(prm_.noise.sigma_gps)};
    }

    double distToTarget() const { return (prm_.target - q_).norm(); }

  private:
    SimParams prm_;
    Rng rng_;
    Vec2 q_;
};

}  // namespace gfs

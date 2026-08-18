#pragma once
// Scenario runners for the GPS-free hidden-target seeking study.
// Each scenario writes CSV files into a results directory.

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include "Estimators.hpp"
#include "Math.hpp"
#include "Model.hpp"

namespace gfs {

struct RunSummary {
    double final_dist = 0.0;      // mean vehicle-to-target distance, last 10 s
    double final_psi_err = 0.0;   // mean |psi error|, last 10 s [rad]
    double time_to_reach = -1.0;  // first time dist < reach_tol held for 5 s
    bool reached = false;
};

struct RunOptions {
    bool excitation = true;
    double psi_hat0 = 0.0;
    double reach_tol = 0.5;
    std::ostream* trace = nullptr;      // optional CSV time-series sink
    double trace_period = 0.2;          // [s]
};

// Run the proposed relative seeker in closed loop. Returns summary stats.
inline RunSummary runRelativeSeeker(const SimParams& prm, unsigned seed,
                                    const RunOptions& opt = {}) {
    World world(prm, seed);
    RelativeSeeker seeker(prm.ctrl, prm.meas_period);
    seeker.setInitialYaw(opt.psi_hat0);
    if (!opt.excitation) seeker.disableExcitation();

    Vec2 q_dr = prm.q0;  // dead-reckoned position (for the drift comparison)
    const int steps = static_cast<int>(prm.T / prm.dt);
    const int meas_every = std::max(1, static_cast<int>(prm.meas_period / prm.dt));
    const int trace_every = std::max(1, static_cast<int>(opt.trace_period / prm.dt));

    RunSummary out;
    double tail_dist = 0.0, tail_psi = 0.0;
    int tail_count = 0;
    double reach_start = -1.0;

    if (opt.trace) {
        *opt.trace << "t,qx,qy,dist,psi_err,e_hat_err,deadreck_err,meas_range\n";
    }

    for (int k = 0; k <= steps; ++k) {
        const double t = k * prm.dt;
        if (k % meas_every == 0) seeker.update(t, world.measure());
        const Vec2 u = seeker.control(t);
        const Vec2 odo = world.step(u);
        seeker.propagate(odo);
        q_dr += odo;

        const double dist = world.distToTarget();
        const double psi_err = std::fabs(wrapAngle(seeker.psiHat() - prm.beacon_yaw));
        const Vec2 e_true = prm.target - world.q();
        const double e_err = (seeker.eHat() - e_true).norm();

        if (opt.trace && k % trace_every == 0) {
            *opt.trace << t << ',' << world.q().x << ',' << world.q().y << ','
                       << dist << ',' << psi_err << ',' << e_err << ','
                       << (q_dr - world.q()).norm() << ',' << seeker.measuredRange()
                       << '\n';
        }

        if (dist < opt.reach_tol) {
            if (reach_start < 0.0) reach_start = t;
            if (!out.reached && t - reach_start >= 5.0) {
                out.reached = true;
                out.time_to_reach = reach_start;
            }
        } else {
            reach_start = -1.0;
        }

        if (t >= prm.T - 10.0) {
            tail_dist += dist;
            tail_psi += psi_err;
            ++tail_count;
        }
    }
    out.final_dist = tail_dist / std::max(1, tail_count);
    out.final_psi_err = tail_psi / std::max(1, tail_count);
    return out;
}

// Closed-loop EKF baseline: steer with u = k_p (p_hat - q_hat) + excitation.
inline RunSummary runEkfSeeker(const SimParams& prm, unsigned seed, bool use_gps,
                               double* abs_target_err_out = nullptr) {
    World world(prm, seed);
    FullEkf ekf(prm, seed ^ 0x9e3779b9u);
    const int steps = static_cast<int>(prm.T / prm.dt);
    const int meas_every = std::max(1, static_cast<int>(prm.meas_period / prm.dt));

    RunSummary out;
    double tail_dist = 0.0, tail_abs = 0.0;
    int tail_count = 0;
    double reach_start = -1.0;

    for (int k = 0; k <= steps; ++k) {
        const double t = k * prm.dt;
        if (k % meas_every == 0) {
            ekf.updateBeacon(world.measure(), prm.noise);
            if (use_gps && prm.noise.sigma_gps > 0.0)
                ekf.updateGps(world.gps(), prm.noise.sigma_gps);
        }
        Vec2 u = ekf.relTarget() * prm.ctrl.k_p;
        const double a = prm.ctrl.exc_amp * std::exp(-prm.ctrl.exc_lambda * t);
        u += Vec2{a * std::cos(prm.ctrl.exc_omega * t),
                  a * std::sin(prm.ctrl.exc_omega * t)};
        u = saturate(u, prm.ctrl.v_max);
        const Vec2 odo = world.step(u);
        ekf.propagate(odo, prm.dt, prm.noise.sigma_odo);

        const double dist = world.distToTarget();
        if (dist < 0.5) {
            if (reach_start < 0.0) reach_start = t;
            if (!out.reached && t - reach_start >= 5.0) {
                out.reached = true;
                out.time_to_reach = reach_start;
            }
        } else {
            reach_start = -1.0;
        }
        if (t >= prm.T - 10.0) {
            tail_dist += dist;
            tail_abs += (ekf.pHat() - prm.target).norm();
            ++tail_count;
        }
    }
    out.final_dist = tail_dist / std::max(1, tail_count);
    if (abs_target_err_out) *abs_target_err_out = tail_abs / std::max(1, tail_count);
    return out;
}

// Closed-loop naive baseline: GPS treated as the true vehicle position.
inline RunSummary runNaiveGpsSeeker(const SimParams& prm, unsigned seed) {
    World world(prm, seed);
    NaiveGpsSeeker seeker(prm.ctrl, prm.meas_period);
    const int steps = static_cast<int>(prm.T / prm.dt);
    const int meas_every = std::max(1, static_cast<int>(prm.meas_period / prm.dt));

    RunSummary out;
    double tail_dist = 0.0;
    int tail_count = 0;
    double reach_start = -1.0;

    for (int k = 0; k <= steps; ++k) {
        const double t = k * prm.dt;
        if (k % meas_every == 0) seeker.update(t, world.measure(), world.gps());
        const Vec2 u = seeker.control(t);
        world.step(u);

        const double dist = world.distToTarget();
        if (dist < 0.5) {
            if (reach_start < 0.0) reach_start = t;
            if (!out.reached && t - reach_start >= 5.0) {
                out.reached = true;
                out.time_to_reach = reach_start;
            }
        } else {
            reach_start = -1.0;
        }
        if (t >= prm.T - 10.0) {
            tail_dist += dist;
            ++tail_count;
        }
    }
    out.final_dist = tail_dist / std::max(1, tail_count);
    return out;
}

inline double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

inline double quantile(std::vector<double> v, double q) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const double idx = q * (v.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(idx);
    const std::size_t hi = std::min(lo + 1, v.size() - 1);
    return v[lo] + (idx - lo) * (v[hi] - v[lo]);
}

}  // namespace gfs

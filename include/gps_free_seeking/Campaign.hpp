#pragma once
// Campaign infrastructure (plan §5): shared trial runner over all methods,
// success criterion, statistics (Wilson, percentile bootstrap), and
// provenance manifests. No trial is ever discarded; failures carry a cause.

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <string>
#include <vector>

#include "Baselines.hpp"
#include "FixedLagSmoother.hpp"
#include "Supervisor.hpp"
#include "WorldSE2.hpp"

namespace gfs {

enum class MethodKind : int {
    kProposed = 0,
    kProposedSmoother = 1,
    kOracleYaw = 2,
    kEkf = 3,
    kEndpointOnly = 4,
    kFixedDecayExc = 5,
    kNoExcitation = 6,
};

inline const char* methodName(MethodKind m) {
    switch (m) {
        case MethodKind::kProposed: return "proposed";
        case MethodKind::kProposedSmoother: return "proposed_smoother";
        case MethodKind::kOracleYaw: return "oracle_yaw";
        case MethodKind::kEkf: return "ekf";
        case MethodKind::kEndpointOnly: return "endpoint_only";
        case MethodKind::kFixedDecayExc: return "fixed_decay_exc";
        case MethodKind::kNoExcitation: return "no_excitation";
    }
    return "?";
}

struct TrialMetrics {
    bool success = false;
    double time_to_goal = -1.0;     // [s] first reach of a successful hold
    double station_rmse = 0.0;      // RMS dist over last 30 s
    double theta_err = 0.0;         // control-path yaw error at end [rad]
    double mean_runtime_us = 0.0;   // estimator update cost
    double deadreck_err = 0.0;      // |integrated odo - true s| at end
    double mean_path_err = 0.0;     // mean |s_est - s_true| at packet times
    int retriggers = 0;
    // Disturbance metrics (populated when a yaw/target step is configured):
    int adoptions = 0;              // change-detection adoptions in the trial
    double detect_delay = -1.0;     // first adoption after step - step time
    double recovery_time = -1.0;    // yaw recovery after a required change [s]
    bool recovery_required = false; // initial post-change error >= cert_off
    bool within_tolerance = false;  // no recovery required at change time
    std::string recovery_class = "not_applicable";
    bool post_success = false;      // success evaluated after the task event
    double post_time_to_goal = -1.0;// relative to the task event [s]
    std::string failure = "";       // "", never_certified, no_reach, hold_broken
};

// Success (plan §5): reach 0.25 m, then remain within 0.35 m for 10 s.
class SuccessTracker {
  public:
    void update(double t, double dist) {
        if (dist <= 0.25 && first_reach_ < 0.0) first_reach_ = t;
        if (dist <= 0.35) {
            if (hold_start_ < 0.0 && first_reach_ >= 0.0) hold_start_ = t;
            if (hold_start_ >= 0.0 && t - hold_start_ >= 10.0) success_ = true;
        } else {
            hold_start_ = -1.0;
        }
    }
    bool success() const { return success_; }
    double timeToGoal() const { return first_reach_; }
    bool reached() const { return first_reach_ >= 0.0; }

  private:
    double first_reach_ = -1.0;
    double hold_start_ = -1.0;
    bool success_ = false;
};

// Runs one closed-loop trial of the selected method against WorldSE2 and
// scores it. Algorithm per control step: emit/deliver packets (with any
// configured dropout, delay, or disturbance), feed the method under test,
// step the world with its command, and update the success tracker; a
// separate post-disturbance tracker starts at the disturbance time so a
// pre-disturbance success latch can never mask a failed recovery.
// Ground truth is touched only by the world, the oracle hook, and the
// evaluator metrics, never by the estimators.
inline TrialMetrics runCampaignTrial(MethodKind kind, const TrialGeometry& geo,
                                     const OdomErrorParams& oe,
                                     const RelayErrorParams& re, unsigned seed,
                                     double T = 150.0,
                                     double yaw_step_time = -1.0,
                                     double yaw_step_size = 0.0,
                                     double target_step_time = -1.0,
                                     Vec2 target_step_offset = Vec2{0, 0}) {
    const double dt = 0.01, meas_period = 0.05;
    WorldSE2 world(geo, oe, re, dt, seed);
    if (yaw_step_time > 0.0) world.setYawStep(yaw_step_time, yaw_step_size);
    if (target_step_time > 0.0)
        world.setTargetStep(target_step_time, target_step_offset);
    const double calibration_event_t = yaw_step_time > 0.0 ? yaw_step_time : -1.0;
    const double task_event_t = target_step_time > 0.0
                                    ? target_step_time
                                    : calibration_event_t;

    SupervisorConfig scfg;
    RegistrationConfig rcfg;
    if (kind == MethodKind::kEndpointOnly) rcfg.endpoint_only = true;
    if (kind == MethodKind::kFixedDecayExc)
        scfg.exc_mode = ExcitationMode::kFixedDecay;
    if (kind == MethodKind::kNoExcitation)
        scfg.exc_mode = ExcitationMode::kNone;

    GfsPipeline pipe(scfg, rcfg);
    EkfPipeline ekf(scfg);
    const bool use_ekf = (kind == MethodKind::kEkf);

    FixedLagSmoother smoother;
    const bool use_smoother = (kind == MethodKind::kProposedSmoother);
    int packets_since_solve = 0;
    double last_pkt_t = 0.0;
    Vec2 last_pkt_s{0, 0};
    bool have_prev_pkt = false;

    Vec2 s_deadreck{0, 0};
    double runtime_sum = 0.0;
    int runtime_n = 0;
    double path_err_sum = 0.0;
    int path_err_n = 0;
    double smooth_path_err_sum = 0.0;
    int smooth_path_err_n = 0;
    SuccessTracker tracker;
    SuccessTracker post_tracker;  // evaluates only t >= task event
    double first_adopt_after_step = -1.0;
    double recovery_t = -1.0;
    bool recovery_state_initialized = false;
    bool recovery_required = false;
    bool within_tolerance = false;
    int adoptions_at_event = 0;
    bool adoption_baseline_captured = false;
    double tail2 = 0.0;
    int tail_n = 0;
    bool ever_certified = false;

    const int steps = static_cast<int>(T / dt);
    const int meas_every = static_cast<int>(meas_period / dt);
    for (int k = 0; k <= steps; ++k) {
        const double t = k * dt;
        if (calibration_event_t > 0.0 && t >= calibration_event_t &&
            !adoption_baseline_captured) {
            adoptions_at_event = use_ekf ? 0 : pipe.adoptions();
            adoption_baseline_captured = true;
        }
        if (k % meas_every == 0) world.emitPacket(t);
        RelayPacket pkt;
        while (world.deliverDue(t, &pkt)) {
            if (use_ekf) {
                ekf.onPacket(pkt);
            } else {
                if (kind == MethodKind::kOracleYaw)
                    pipe.setOracleYaw(world.trueTheta());
                pipe.onPacket(pkt);
                if (use_smoother) {
                    FixedLagSmoother::Node nd;
                    nd.t = pkt.t;
                    nd.s_init = pipe.s();
                    nd.pkt = pkt;
                    nd.odo_delta =
                        have_prev_pkt ? pipe.s() - last_pkt_s : Vec2{0, 0};
                    // Per-node odometry sigma follows the SWEPT world noise
                    // (P0 fix: was hard-coded 0.01 regardless of the sweep):
                    // increments accumulate over meas_period/dt control steps.
                    nd.odo_sigma =
                        oe.sigma_xy * std::sqrt(meas_period / dt) + 1e-6;
                    nd.dt_prev = have_prev_pkt ? pkt.t - last_pkt_t : 0.0;
                    // Drop late out-of-order packets for the smoother: its
                    // odometry chain requires time-ordered nodes (the
                    // registration window handles reordering by sorted
                    // insertion; documented engineering choice).
                    if (!have_prev_pkt || pkt.t > last_pkt_t) {
                        smoother.addNode(nd);
                        last_pkt_t = pkt.t;
                        last_pkt_s = pipe.s();
                        have_prev_pkt = true;
                    }
                    if (++packets_since_solve >= 5 && pipe.estimate().valid) {
                        packets_since_solve = 0;
                        const auto& est = pipe.estimate();
                        const Vec2 x0 =
                            pipe.s() - rotate(est.theta_hat, pkt.lv());
                        const Vec2 p0 = x0 + rotate(est.theta_hat, pkt.lt());
                        smoother.setInitialGuess(est.theta_hat, x0, p0);
                        // Solve cost is part of the method's runtime (P0 fix).
                        const auto ts0 = std::chrono::steady_clock::now();
                        const bool ok = smoother.solve();
                        runtime_sum +=
                            std::chrono::duration<double, std::micro>(
                                std::chrono::steady_clock::now() - ts0)
                                .count();
                        if (ok) {
                            pipe.acceptSmoother(smoother.theta(),
                                                smoother.thetaVariance());
                            // Smoothed-pose path error (P0 fix: pipe.s() does
                            // not measure the smoother's trajectory).
                            smooth_path_err_sum +=
                                (smoother.lastPose() - world.sTrue()).norm();
                            ++smooth_path_err_n;
                        }
                    }
                }
            }
            const auto& e = use_ekf ? ekf.estimate() : pipe.estimate();
            runtime_sum += e.runtime_us;
            ++runtime_n;
            ever_certified = ever_certified || e.certified;
            const Vec2 s_est = use_ekf ? ekf.sHat() : pipe.s();
            path_err_sum += (s_est - world.sTrue()).norm();
            ++path_err_n;
        }
        const SeekCommand& cmd = use_ekf ? ekf.command() : pipe.command();
        const OdometrySample o = world.step(cmd, t);
        s_deadreck += rotate((use_ekf ? ekf.headingO() : pipe.headingO()),
                             o.dxy);  // naive integration diagnostic
        if (use_ekf) ekf.onOdometry(o);
        else pipe.onOdometry(o);

        const double dist = world.distToTargetAt(t);
        tracker.update(t, dist);
        if (task_event_t > 0.0 && t >= task_event_t)
            post_tracker.update(t, dist);
        if (calibration_event_t > 0.0 && t >= calibration_event_t) {
            if (!use_ekf) {
                if (first_adopt_after_step < 0.0 &&
                    pipe.lastAdoptTime() >= calibration_event_t)
                    first_adopt_after_step = pipe.lastAdoptTime();
                const double yaw_err = std::fabs(wrapAngle(
                    pipe.thetaCtrl() - world.trueThetaAt(t)));
                if (!recovery_state_initialized) {
                    recovery_required = yaw_err >= scfg.cert_off_rad;
                    within_tolerance = !recovery_required;
                    recovery_state_initialized = true;
                }
                if (recovery_required && recovery_t < 0.0 &&
                    t > calibration_event_t &&
                    yaw_err < scfg.cert_off_rad)
                    recovery_t = t;
            }
        }
        if (t >= T - 30.0) {
            tail2 += dist * dist;
            ++tail_n;
        }
    }

    TrialMetrics m;
    m.success = tracker.success();
    m.time_to_goal = tracker.timeToGoal();
    m.station_rmse = std::sqrt(tail2 / std::max(1, tail_n));
    const double th_hat =
        use_ekf ? ekf.estimate().theta_hat : pipe.thetaCtrl();
    m.theta_err = std::fabs(wrapAngle(th_hat - world.trueThetaAt(T)));
    m.mean_runtime_us = runtime_n ? runtime_sum / runtime_n : 0.0;
    m.deadreck_err = (s_deadreck - world.sTrue()).norm();
    // For the smoother method the path metric is the SMOOTHED pose error;
    // integrated-odometry error is reported separately via deadreck_err.
    m.mean_path_err = (use_smoother && smooth_path_err_n)
                          ? smooth_path_err_sum / smooth_path_err_n
                          : (path_err_n ? path_err_sum / path_err_n : 0.0);
    m.retriggers = use_ekf ? ekf.retriggers() : pipe.retriggers();
    if (!m.success) {
        if (!ever_certified) m.failure = "never_certified";
        else if (!tracker.reached()) m.failure = "no_reach";
        else m.failure = "hold_broken";
    }
    if (task_event_t > 0.0) {
        m.post_success = post_tracker.success();
        m.post_time_to_goal = post_tracker.timeToGoal() >= 0.0
                                  ? post_tracker.timeToGoal() - task_event_t
                                  : -1.0;
    }
    if (calibration_event_t > 0.0) {
        m.adoptions = use_ekf ? 0 : pipe.adoptions() - adoptions_at_event;
        m.detect_delay = first_adopt_after_step >= 0.0
                             ? first_adopt_after_step - calibration_event_t
                             : -1.0;
        m.recovery_time =
            recovery_t >= 0.0 ? recovery_t - calibration_event_t : -1.0;
        m.recovery_required = recovery_required;
        m.within_tolerance = within_tolerance;
        if (within_tolerance) {
            m.recovery_class = "within_tolerance";
        } else if (m.recovery_time < 0.0) {
            m.recovery_class = "not_recovered";
        } else if (m.detect_delay >= 0.0 &&
                   m.detect_delay <= m.recovery_time) {
            m.recovery_class = "detected_recovered";
        } else {
            m.recovery_class = "refined_recovered";
        }
    } else {
        m.adoptions = use_ekf ? 0 : pipe.adoptions();  // false-alarm count
    }
    return m;
}

// ---- Statistics -----------------------------------------------------------

struct WilsonInterval {
    double lo, hi;
};

inline WilsonInterval wilson95(int successes, int n) {
    if (n == 0) return {0.0, 1.0};
    const double z = 1.96, p = static_cast<double>(successes) / n;
    const double d = 1.0 + z * z / n;
    const double c = p + z * z / (2.0 * n);
    const double h = z * std::sqrt(p * (1 - p) / n + z * z / (4.0 * n * n));
    return {std::max(0.0, (c - h) / d), std::min(1.0, (c + h) / d)};
}

// Percentile bootstrap CI of the median (fixed internal seed: deterministic).
inline void bootstrapMedianCi(const std::vector<double>& x, double* lo,
                              double* hi, int reps = 1000) {
    if (x.empty()) {
        *lo = *hi = 0.0;
        return;
    }
    Rng rng(1234567u);
    std::vector<double> meds;
    meds.reserve(reps);
    std::vector<double> samp(x.size());
    for (int r = 0; r < reps; ++r) {
        for (size_t i = 0; i < x.size(); ++i)
            samp[i] = x[static_cast<size_t>(rng.uni(0.0, 1.0) * x.size()) %
                        x.size()];
        std::sort(samp.begin(), samp.end());
        meds.push_back(samp[samp.size() / 2]);
    }
    std::sort(meds.begin(), meds.end());
    *lo = meds[static_cast<size_t>(0.025 * reps)];
    *hi = meds[static_cast<size_t>(0.975 * reps)];
}

inline double medianOf(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

inline double quantileOf(std::vector<double> v, double q) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const double idx = q * (v.size() - 1);
    const size_t lo = static_cast<size_t>(idx);
    const size_t hi = std::min(lo + 1, v.size() - 1);
    return v[lo] + (idx - lo) * (v[hi] - v[lo]);
}

// Provenance manifest (plan §5): every figure/table value must trace here.
// P1 fix: the manifest dumps the ACTUAL instantiated configuration; it no
// longer claims an ini file that the binary does not read (default.ini is
// human documentation of the same values, cross-checked at Gate C freeze).
// Compile identity, filled at build time. A binary built from dirty or
// different sources is distinguishable even when the caller's hash matches.
#ifndef GFS_BUILD_MODE
#define GFS_BUILD_MODE "unspecified"
#endif
#ifndef GFS_GIT_HEAD
#define GFS_GIT_HEAD "unspecified"
#endif
#ifndef GFS_CONFIGURE_DIRTY
#define GFS_CONFIGURE_DIRTY "unknown"
#endif
// Git working-tree cleanliness at run time; set by the driver from argv so a
// binary built from uncommitted sources is flagged in every manifest.
inline std::string& gitDirtyFlag() {
    static std::string flag = "unknown";
    return flag;
}
inline std::string& runIdentity() {
    static std::string id = "unspecified";
    return id;
}
inline const char* buildStamp() { return __DATE__ " " __TIME__; }
inline const char* compilerId() {
#if defined(__clang__)
    return "clang " __clang_version__;
#elif defined(__GNUC__)
    return "gcc " __VERSION__;
#elif defined(_MSC_VER)
    return "msvc";
#else
    return "unknown";
#endif
}

inline void writeManifest(const std::string& path, const std::string& study,
                          const std::string& git_head, int n_seeds,
                          unsigned seed_base, double T,
                          const std::string& extra_params = "") {
    const SupervisorConfig sc;
    const RegistrationConfig rc;
    std::ofstream f(path);
    f << "{\n"
      << "  \"study\": \"" << study << "\",\n"
      << "  \"run_id\": \"" << runIdentity() << "\",\n"
      << "  \"timestamp_utc\": " << static_cast<long long>(std::time(nullptr))
      << ",\n"
      << "  \"git_head\": \"" << git_head << "\",\n"
      << "  \"compiled_git_head\": \"" << GFS_GIT_HEAD << "\",\n"
      << "  \"git_dirty\": \"" << gitDirtyFlag() << "\",\n"
      << "  \"configure_dirty\": \"" << GFS_CONFIGURE_DIRTY << "\",\n"
      << "  \"build_stamp\": \"" << buildStamp() << "\",\n"
      << "  \"compiler\": \"" << compilerId() << "\",\n"
      << "  \"build_mode\": \"" << GFS_BUILD_MODE << "\",\n"
      << "  \"n_seeds\": " << n_seeds << ",\n"
      << "  \"seed_base\": " << seed_base << ",\n"
      << "  \"horizon_s\": " << T << ",\n"
      << "  \"supervisor\": {\"cert_on_rad\": " << sc.cert_on_rad
      << ", \"cert_off_rad\": " << sc.cert_off_rad
      << ", \"min_correlation\": " << sc.min_correlation
      << ", \"min_window_packets\": " << sc.min_window_packets
      << ", \"consecutive_windows\": " << sc.consecutive_windows
      << ", \"min_dwell\": " << sc.min_dwell
      << ", \"v_excite\": " << sc.v_excite
      << ", \"omega_excite\": " << sc.omega_excite
      << ", \"k_p\": " << sc.k_p << ", \"k_omega\": " << sc.k_omega
      << ", \"v_max\": " << sc.v_max << ", \"omega_max\": " << sc.omega_max
      << ", \"maintain_enter\": " << sc.maintain_enter
      << ", \"maintain_exit\": " << sc.maintain_exit
      << ", \"k_e\": " << sc.k_e
      << ", \"theta_drift_rate\": " << sc.theta_drift_rate
      << ", \"change_chi_thresh\": " << sc.change_chi_thresh
      << ", \"change_adopt_after\": " << sc.change_adopt_after
      << ", \"change_min_duration_s\": " << sc.change_min_duration
      << ", \"change_min_interval_s\": " << sc.change_min_interval
      << ", \"exc_lambda_per_s\": " << sc.exc_lambda << "},\n"
      << "  \"registration\": {\"max_packets\": " << rc.max_packets
      << ", \"max_age\": " << rc.max_age
      << ", \"min_packets\": " << rc.min_packets
      << ", \"min_spread\": " << rc.min_spread
      << ", \"endpoint_only\": " << (rc.endpoint_only ? "true" : "false")
      << "}"
      << (extra_params.empty() ? "" : ",\n  " + extra_params) << "\n"
      << "}\n";
}

}  // namespace gfs

#pragma once
// RelativeTargetFilter, ExcitationSupervisor, nonholonomic seek controller,
// and the GfsPipeline that wires them (plan §4.1 components 2-3, §4.2).
//
// The pipeline consumes only OdometrySamples and RelayPackets; it never sees
// ground truth. This boundary is enforced by construction here and audited by
// integration tests (plan §10).

#include <deque>

#include "Math.hpp"
#include "Registration.hpp"
#include "Types.hpp"

namespace gfs {

enum class ExcitationMode : int {
    kSupervisedArc = 0,  // constant-curvature arc until certified (default)
    kFixedDecay = 1,     // decaying arc, no supervision (ablation)
    kNone = 2            // no exploratory motion at all (necessity check)
};

struct SupervisorConfig {
    // Certificate (plan §4.2): 1.96*sqrt(var_theta) <= cert_on to certify;
    // re-EXCITE above cert_off (hysteresis), or on low correlation / thin
    // window. Thresholds live in config and are swept in sensitivity tests.
    double cert_on_rad = 10.0 * kPi / 180.0;
    double cert_off_rad = 15.0 * kPi / 180.0;
    // Gross-mismatch detector only. The variance certificate is the quality
    // gate; healthy transits at long relay range run corr ~ 0.85-0.95, so a
    // tight correlation threshold falsely retriggers EXCITE (root cause of
    // the goal-approach thrash found 2026-08-16; see execution log).
    double min_correlation = 0.5;
    int min_window_packets = 8;
    int consecutive_windows = 3;   // certified windows required to leave EXCITE
    double min_dwell = 1.0;        // [s] minimum time in a mode (anti-chatter)

    // EXCITE arc.
    double v_excite = 0.6;         // [m/s]
    double omega_excite = 0.45;    // [rad/s]
    ExcitationMode exc_mode = ExcitationMode::kSupervisedArc;
    double exc_lambda = 0.05;      // [1/s] decay rate for kFixedDecay

    // SEEK law and saturation (one final saturation block for all modes).
    double k_p = 0.8;
    double k_omega = 2.0;
    double v_max = 1.0;            // [m/s]
    double omega_max = 1.2;        // [rad/s]

    // MAINTAIN band.
    double maintain_enter = 0.20;  // [m]
    double maintain_exit = 0.30;   // [m]

    // Relative target filter gain.
    double k_e = 2.0;              // [1/s]

    // Unmodeled relay-yaw random-walk intensity [rad/sqrt(s)]: the
    // control-path variance grows as var += rate^2 * dt (standard
    // random-walk model; the earlier (rate*dt)^2-per-packet accumulation
    // was neither deterministic-rate nor random-walk and drastically
    // undercounted -- P1 finding, 2026-08-16 review). Retriggering is
    // driven by this variance, not by the raw window variance, which
    // necessarily starves whenever the vehicle slows.
    double theta_drift_rate = 0.002;

    // Calibration change detection: adopt the current window estimate when
    // it is inconsistent with the control path (chi-square > thresh) for
    // several consecutive CERTIFIED windows -- e.g. the relay was physically
    // rotated. Only certified windows may vote (junk windows under heavy
    // odometry noise carry correlated pose errors the first-order
    // certificate underestimates; trusting them caused control-yaw thrash,
    // 2026-08-16 trace), adoption requires persistence and is rate-limited.
    double change_chi_thresh = 9.0;   // (3-sigma)^2
    int change_adopt_after = 10;      // consecutive inconsistent cert windows
    double change_min_duration = 4.0; // [s] inconsistency must persist through
                                      // one full window turnover; overlapping
                                      // estimates are not independent votes
    double change_min_interval = 2.0; // [s] between adoptions
};

// Tracks the task vector e in O: propagates with odometry, corrects with the
// gauge-invariant packet readout R(theta_hat) d_k.
class RelativeTargetFilter {
  public:
    explicit RelativeTargetFilter(double k_e) : k_e_(k_e) {}

    void propagate(const Vec2& ds_O) {
        if (initialized_) e_hat_ -= ds_O;
    }

    void correct(const Vec2& e_meas, double dt_meas) {
        if (!initialized_) {
            e_hat_ = e_meas;
            initialized_ = true;
            return;
        }
        const double alpha = 1.0 - std::exp(-k_e_ * std::max(dt_meas, 1e-4));
        e_hat_ += alpha * (e_meas - e_hat_);
    }

    bool initialized() const { return initialized_; }
    const Vec2& eHat() const { return e_hat_; }

  private:
    double k_e_;
    Vec2 e_hat_{0, 0};
    bool initialized_ = false;
};

// EXCITE-SEEK-MAINTAIN hybrid supervisor with hysteresis and dwell.
//
// Mode semantics:
//   EXCITE   drive the constant-curvature acquisition arc until the yaw
//            certificate holds on `consecutive_windows` certified windows.
//   SEEK     saturated pursuit of the filtered task vector; falls back to
//            EXCITE only if the CONTROL-PATH certificate is lost (window
//            starvation alone can never demote a valid calibration).
//   MAINTAIN station keeping inside `maintain_enter`; re-seeks past
//            `maintain_exit`. Never retriggers on certificate decay,
//            because at e ~ 0 yaw error is task-irrelevant by the
//            quotient identity e = R(theta) d.
class ExcitationSupervisor {
  public:
    explicit ExcitationSupervisor(const SupervisorConfig& cfg) : cfg_(cfg) {}

    // One supervisor step. Inputs: the newest estimator output (its
    // theta_variance is the CONTROL-PATH variance), the current task
    // distance, and time for dwell accounting. Certification requires a
    // healthy current window (acquisition gate); loss depends only on the
    // control-path variance (see class comment). Returns the mode and
    // flags SEEK/MAINTAIN -> EXCITE transitions in *retriggered.
    Mode update(double t, const EstimatorOutput& est, double dist_to_goal,
                bool* retriggered) {
        *retriggered = false;
        const double halfwidth =
            1.96 * std::sqrt(std::max(est.theta_variance, 0.0));
        // Acquisition (cert_ok) additionally requires a healthy CURRENT
        // window (correlation, packet count); loss (cert_lost) depends only
        // on the control-path variance -- a starved or noise-dominated
        // current window must not invalidate a frozen valid calibration.
        const bool cert_ok = est.valid && halfwidth <= cfg_.cert_on_rad &&
                             est.correlation >= cfg_.min_correlation &&
                             est.window_packets >= cfg_.min_window_packets;
        const bool cert_lost = !est.valid || halfwidth > cfg_.cert_off_rad;

        if (cert_ok) ++consecutive_ok_;
        else consecutive_ok_ = 0;

        const bool dwell_over = (t - mode_since_) >= cfg_.min_dwell;

        switch (mode_) {
            case Mode::kExcite:
                if (dwell_over && consecutive_ok_ >= cfg_.consecutive_windows)
                    enter(Mode::kSeek, t);
                break;
            case Mode::kSeek:
                if (cert_lost && dwell_over) {
                    enter(Mode::kExcite, t);
                    *retriggered = true;
                } else if (dist_to_goal <= cfg_.maintain_enter && dwell_over) {
                    enter(Mode::kMaintain, t);
                }
                break;
            case Mode::kMaintain:
                // Certificate decay is EXPECTED at station keeping: motion
                // stops, the window starves, information drains. Yaw error is
                // harmless while e ~ 0 (e_meas = R(theta)d stays small under
                // any yaw), so MAINTAIN never retriggers on certificate loss;
                // it re-seeks only if the task vector itself drifts out.
                // (Execution-log decision 2026-08-16; prevents goal-area
                // EXCITE thrash observed in first closed-loop runs.)
                if (dist_to_goal >= cfg_.maintain_exit && dwell_over) {
                    enter(Mode::kSeek, t);
                }
                break;
        }
        return mode_;
    }

    Mode mode() const { return mode_; }
    int retriggers() const { return retrigger_count_; }

  private:
    void enter(Mode m, double t) {
        if (m == Mode::kExcite && mode_ != Mode::kExcite) ++retrigger_count_;
        mode_ = m;
        mode_since_ = t;
        consecutive_ok_ = 0;
    }

    SupervisorConfig cfg_;
    Mode mode_ = Mode::kExcite;
    double mode_since_ = 0.0;
    int consecutive_ok_ = 0;
    int retrigger_count_ = 0;
};

// Nonholonomic seek law (plan §4.2). All commands pass through one final
// saturation block; the paper equation, this code, and the proof match.
inline SeekCommand seekCommand(Mode mode, const Vec2& e_hat, double heading_O,
                               const SupervisorConfig& cfg, bool retriggered,
                               double t = 0.0) {
    SeekCommand c;
    c.mode = mode;
    c.retriggered = retriggered;
    switch (mode) {
        case Mode::kExcite:
            switch (cfg.exc_mode) {
                case ExcitationMode::kSupervisedArc:
                    c.v = cfg.v_excite;
                    c.omega = cfg.omega_excite;
                    break;
                case ExcitationMode::kFixedDecay:
                    c.v = cfg.v_excite * std::exp(-cfg.exc_lambda * t);
                    c.omega = cfg.omega_excite;
                    break;
                case ExcitationMode::kNone:
                    c.v = 0.0;
                    c.omega = 0.0;
                    break;
            }
            break;
        case Mode::kSeek: {
            const double alpha =
                wrapAngle(std::atan2(e_hat.y, e_hat.x) - heading_O);
            c.v = std::min(cfg.v_max, cfg.k_p * e_hat.norm()) *
                  std::max(0.0, std::cos(alpha));
            c.omega = cfg.k_omega * alpha;
            break;
        }
        case Mode::kMaintain:
            c.v = 0.0;
            c.omega = 0.0;
            break;
    }
    // Final saturation block (single place, all modes).
    c.v = std::max(-cfg.v_max, std::min(cfg.v_max, c.v));
    c.omega = std::max(-cfg.omega_max, std::min(cfg.omega_max, c.omega));
    return c;
}

// Full vehicle-side pipeline: odometry integration in O, pose history for
// delayed packets, registration, filter, supervisor, controller.
class GfsPipeline {
  public:
    GfsPipeline(const SupervisorConfig& scfg, const RegistrationConfig& rcfg)
        : cfg_(scfg), reg_(rcfg), filter_(scfg.k_e), sup_(scfg) {
        // Anchor interpolation for packets measured before the first
        // odometry increment but delivered later.
        pose_hist_.push_back({0.0, Vec2{0, 0}, 0.0});
    }

    // Every control step: integrate body-frame odometry into O using the
    // midpoint-heading rule, accumulate the pose variance that feeds the
    // correlated-odometry certificate, propagate the task filter, and log
    // the pose history used to time-align delayed packets.
    void onOdometry(const OdometrySample& o) {
        const double h_mid = heading_O_ + 0.5 * o.dtheta;
        heading_O_ = wrapAngle(heading_O_ + o.dtheta);
        const Vec2 ds = rotate(h_mid, o.dxy);  // body -> O
        s_ += ds;
        cum_pose_var_ += o.sigma_xy * o.sigma_xy;  // per-axis, isotropic
        filter_.propagate(ds);
        t_ = o.t;
        pose_hist_.push_back({o.t, s_, cum_pose_var_});
        while (pose_hist_.size() > 4096 ||
               (pose_hist_.size() > 2 && o.t - pose_hist_.front().t > 8.0))
            pose_hist_.pop_front();
    }

    // On packet delivery (possibly delayed, jittered, or reordered).
    // Algorithm:
    //   1. Register the view against the odometry pose AND pose variance
    //      interpolated at the packet's MEASUREMENT timestamp.
    //   2. Solve the windowed registration; optionally accept the fixed-lag
    //      refinement through the variance + chi-square gate.
    //   3. Update the control-path yaw through exactly one of three paths:
    //      initial acquisition / consistent refinement / persistent
    //      certified inconsistency (change detection).
    //   4. Correct the task filter with the delay-compensated measurement
    //      R(theta_ctrl) d_k minus the vehicle motion since packet time.
    //   5. Run the supervisor and produce the saturated command.
    void onPacket(const RelayPacket& pkt) {
        if (!pkt.valid) return;
        reg_.addView(pkt.t, poseAt(pkt.t), pkt, poseVarAt(pkt.t));

        EstimatorOutput est;
        est.window_packets = reg_.windowSize();
        double th, var, info, spread, corr;
        if (reg_.solve(&th, &var, &info, &spread, &corr)) {
            est.valid = true;
            est.theta_hat = th;
            est.info_theta = info;
            est.theta_variance = var;  // estimator delivery variance
            est.spread = spread;
            est.correlation = corr;
            est.runtime_us = reg_.lastRuntimeUs();
        }
        // Optional externally-computed refinement (fixed-lag smoother) is
        // injected via acceptSmoother(); acceptance gate: variance must beat
        // the registration certificate AND the estimate must be
        // chi-consistent with it (guards against a diverged smoother whose
        // self-reported variance is overconfident -- same trap as the EKF
        // inconsistency finding).
        if (est.valid && smoother_valid_ &&
            smoother_var_ < est.theta_variance) {
            const double d = wrapAngle(smoother_theta_ - est.theta_hat);
            const double chi =
                d * d / std::max(est.theta_variance + smoother_var_, 1e-12);
            if (chi <= 9.0) {
                est.theta_hat = smoother_theta_;
                est.theta_variance = smoother_var_;
                est.smoother_active = true;
            }
        }
        // Control-path yaw: best-ever certified estimate. For the static
        // relay of Assumption 1, yaw knowledge does not decay when the window
        // starves (e.g. slowing near the goal); var_ctrl_ inflates only at
        // the configured unmodeled drift rate. Junk windows can never corrupt
        // the control path because they only replace it when strictly more
        // informative.
        // The filter and drift model live at delivery time after delay
        // compensation. Delivery time is monotone even when jitter causes
        // packet timestamps to arrive out of order.
        const double dt_update = have_update_time_
                                     ? std::max(t_ - last_update_t_, 0.0)
                                     : 0.0;
        var_ctrl_ +=
            cfg_.theta_drift_rate * cfg_.theta_drift_rate * dt_update;
        // Control-path update, restructured per review 2026-08-16 #3 into
        // three EXCLUSIVE paths. Uncertified estimates can never touch the
        // control path (the old lower-variance shortcut bypassed every gate).
        const bool window_certified =
            est.valid &&
            1.96 * std::sqrt(std::max(est.theta_variance, 0.0)) <=
                cfg_.cert_on_rad &&
            est.correlation >= cfg_.min_correlation &&
            est.window_packets >= cfg_.min_window_packets;
        if (window_certified) {
            if (!have_ctrl_theta_) {
                // (a) Initial acquisition.
                theta_ctrl_ = est.theta_hat;
                var_ctrl_ = est.theta_variance;
                have_ctrl_theta_ = true;
            } else {
                const double d = wrapAngle(est.theta_hat - theta_ctrl_);
                const double chi =
                    d * d / std::max(est.theta_variance + var_ctrl_, 1e-12);
                if (chi <= cfg_.change_chi_thresh) {
                    // (b) Consistent refinement: keep the better variance.
                    change_count_ = 0;
                    change_since_ = -1.0;
                    if (est.theta_variance < var_ctrl_) {
                        theta_ctrl_ = est.theta_hat;
                        var_ctrl_ = est.theta_variance;
                    }
                } else {
                    // (c) Inconsistent candidate: change-detection path only.
                    // Require a coherent candidate throughout a full window
                    // turnover. An uncertified window or a mutually
                    // inconsistent candidate breaks the streak.
                    if (change_since_ < 0.0) {
                        change_since_ = t_;
                        change_count_ = 0;
                        change_candidate_theta_ = est.theta_hat;
                        change_candidate_var_ = est.theta_variance;
                    } else {
                        const double dc = wrapAngle(
                            est.theta_hat - change_candidate_theta_);
                        const double candidate_chi =
                            dc * dc /
                            std::max(est.theta_variance +
                                         change_candidate_var_,
                                     1e-12);
                        if (candidate_chi > cfg_.change_chi_thresh) {
                            change_since_ = t_;
                            change_count_ = 0;
                        }
                        change_candidate_theta_ = est.theta_hat;
                        change_candidate_var_ = est.theta_variance;
                    }
                    if (++change_count_ >= cfg_.change_adopt_after &&
                        t_ - change_since_ >= cfg_.change_min_duration &&
                        t_ - last_adopt_t_ >= cfg_.change_min_interval) {
                        theta_ctrl_ = est.theta_hat;
                        var_ctrl_ = est.theta_variance;
                        change_count_ = 0;
                        change_since_ = -1.0;
                        last_adopt_t_ = t_;
                        ++adoptions_;
                    }
                }
            }
        } else {
            // "Consecutive certified windows" is literal: missing or
            // uncertified evidence cannot preserve a pending change vote.
            change_count_ = 0;
            change_since_ = -1.0;
        }
        if (have_ctrl_theta_) {
            // Delay compensation (review #5): R(theta)d_k is the task vector
            // at MEASUREMENT time; subtract the vehicle displacement between
            // packet time and now before correcting the current state.
            const Vec2 motion_since = s_ - poseAt(pkt.t);
            const Vec2 e_meas = rotate(theta_ctrl_, pkt.d()) - motion_since;
            est.e_raw = e_meas;
            filter_.correct(e_meas, dt_update);
        }
        last_update_t_ = t_;
        have_update_time_ = true;
        est.e_hat = filter_.eHat();
        // The supervisor certificate reads the CONTROL-PATH variance; the
        // window-level info/variance remain reported for analysis.
        est.theta_variance = var_ctrl_;
        est.valid = have_ctrl_theta_;
        est.certified =
            est.valid &&
            1.96 * std::sqrt(std::max(var_ctrl_, 0.0)) <= cfg_.cert_on_rad;

        bool retrig = false;
        const double dist = filter_.initialized() ? filter_.eHat().norm() : 1e9;
        const Mode m = sup_.update(t_, est, dist, &retrig);
        cmd_ = seekCommand(m, filter_.eHat(), heading_O_, cfg_, retrig, t_);
        last_est_ = est;
    }

    // ORACLE BASELINE HOOK (plan §4.3): pins the control-path yaw to the
    // supplied value with negligible variance. Only the evaluator may call
    // this; it is the labeled oracle bound, never a deployable mode.
    void setOracleYaw(double theta_true) {
        theta_ctrl_ = theta_true;
        var_ctrl_ = 1e-10;
        have_ctrl_theta_ = true;
    }

    // Fixed-lag smoother interface (acceptance gate applied in onPacket).
    void acceptSmoother(double theta, double variance) {
        smoother_theta_ = theta;
        smoother_var_ = variance;
        smoother_valid_ = true;
    }
    void invalidateSmoother() { smoother_valid_ = false; }

    const SeekCommand& command() const { return cmd_; }
    const EstimatorOutput& estimate() const { return last_est_; }
    // Control-path yaw (best-ever certified). Use THIS for yaw-error metrics;
    // estimate().theta_hat is the raw current-window solution, which is
    // meaningless when the window is starved (e.g. at station keeping).
    double thetaCtrl() const { return theta_ctrl_; }
    double controlVariance() const { return var_ctrl_; }
    int adoptions() const { return adoptions_; }
    double lastAdoptTime() const { return last_adopt_t_; }
    int pendingChangeVotes() const { return change_count_; }
    const Vec2& s() const { return s_; }
    double headingO() const { return heading_O_; }
    int retriggers() const { return sup_.retriggers(); }
    Mode mode() const { return sup_.mode(); }

  private:
    Vec2 poseAt(double t) const {
        if (pose_hist_.empty()) return s_;
        if (t <= pose_hist_.front().t) return pose_hist_.front().s;
        for (size_t i = 1; i < pose_hist_.size(); ++i) {
            if (pose_hist_[i].t >= t) {
                const auto& a = pose_hist_[i - 1];
                const auto& b = pose_hist_[i];
                const double u = (t - a.t) / std::max(b.t - a.t, 1e-9);
                return a.s + (b.s - a.s) * u;
            }
        }
        return s_;
    }

    struct Stamped {
        double t;
        Vec2 s;
        double pose_var;
    };

    double poseVarAt(double t) const {
        if (pose_hist_.empty()) return cum_pose_var_;
        if (t <= pose_hist_.front().t) return pose_hist_.front().pose_var;
        for (size_t i = 1; i < pose_hist_.size(); ++i)
            if (pose_hist_[i].t >= t) return pose_hist_[i].pose_var;
        return cum_pose_var_;
    }

    SupervisorConfig cfg_;
    WeightedWindowRegistration reg_;
    RelativeTargetFilter filter_;
    ExcitationSupervisor sup_;
    std::deque<Stamped> pose_hist_;
    Vec2 s_{0, 0};
    double heading_O_ = 0.0;
    double cum_pose_var_ = 0.0;
    double t_ = 0.0;
    double last_update_t_ = 0.0;
    bool have_update_time_ = false;
    SeekCommand cmd_;
    EstimatorOutput last_est_;
    double theta_ctrl_ = 0.0;
    double var_ctrl_ = 1e9;
    bool have_ctrl_theta_ = false;
    int change_count_ = 0;
    double change_since_ = -1.0;
    double change_candidate_theta_ = 0.0;
    double change_candidate_var_ = 1e9;
    double last_adopt_t_ = -1e9;
    int adoptions_ = 0;
    double smoother_theta_ = 0.0;
    double smoother_var_ = 1e9;
    bool smoother_valid_ = false;
};

}  // namespace gfs

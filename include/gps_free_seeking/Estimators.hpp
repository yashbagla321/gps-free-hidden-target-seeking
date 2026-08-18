#pragma once
// Estimators and controllers for GPS-free hidden-target seeking.
//
// RelativeSeeker : the proposed gauge-quotient adaptive estimator/controller.
//                  Uses only beacon-frame range-bearing pairs and compass-
//                  aligned odometry increments. Never uses absolute position.
// FullEkf        : absolute-state EKF baseline on [q, p, x, psi] fusing
//                  odometry, beacon measurements, and (optionally) noisy GPS.
// NaiveGpsSeeker : treats noisy GPS fixes as the true vehicle position and
//                  runs the known-position calibration logic of the baseline
//                  paper on top of them.

#include <deque>

#include "Math.hpp"
#include "Model.hpp"

namespace gfs {

// ---------------------------------------------------------------------------
// Proposed method: adaptive relative seeking on the translation-gauge quotient.
// State: psi_hat (relay yaw in the control frame) and e_hat (target relative
// vector in the control frame). Dynamics used by the observer:
//   e_dot = -u  (static target),   b_v_dot = R(psi)^T u  (static relay).
// Yaw adaptation is a normalized gradient descent on the windowed displacement
// matching cost  J = 0.5 || db_v - R(psi_hat)^T dq_odo ||^2.
// ---------------------------------------------------------------------------
class RelativeSeeker {
  public:
    RelativeSeeker(const ControlParams& c, double meas_period)
        : c_(c), meas_period_(meas_period) {}

    // Called every integration step with the odometry increment.
    void propagate(const Vec2& odo_increment) {
        e_hat_ -= odo_increment;
        cum_odo_ += odo_increment;
    }

    // Called at measurement ticks.
    void update(double t, const BeaconMeasurement& m) {
        const Vec2 bv = m.bv();
        const Vec2 d = m.bt() - bv;  // = R(psi)^T (p - q) + noise

        // --- Yaw adaptation over a sliding displacement window -------------
        hist_.push_back({t, bv, cum_odo_});
        while (hist_.size() > 2 && t - hist_.front().t > c_.window) hist_.pop_front();
        const Vec2 dq = cum_odo_ - hist_.front().cum_odo;
        if (dq.norm() >= c_.d_min) {
            const Vec2 db = bv - hist_.front().bv;
            const Vec2 pred = rotateT(psi_hat_, dq);        // R(psi_hat)^T dq
            const Vec2 res = db - pred;
            // d/dpsi [R(psi)^T dq] = R(psi)^T S^T dq, with S = [[0,-1],[1,0]].
            const Vec2 grad_dir = rotateT(psi_hat_, Vec2{dq.y, -dq.x});
            const double denom = dq.dot(dq) + c_.disp_reg;
            psi_hat_ = wrapAngle(psi_hat_ + c_.gamma_psi * meas_period_ *
                                                res.dot(grad_dir) / denom);
            adapted_ = true;
        }

        // --- Relative target observer --------------------------------------
        const Vec2 e_meas = rotate(psi_hat_, d);            // R(psi_hat) d
        const double alpha = 1.0 - std::exp(-c_.k_e * meas_period_);
        e_hat_ += alpha * (e_meas - e_hat_);
        range_to_target_ = d.norm();                        // gauge-invariant readout
    }

    Vec2 control(double t) const {
        Vec2 u = e_hat_ * c_.k_p;
        const double a = c_.exc_amp * std::exp(-c_.exc_lambda * t);
        u += Vec2{a * std::cos(c_.exc_omega * t), a * std::sin(c_.exc_omega * t)};
        return saturate(u, c_.v_max);
    }

    double psiHat() const { return psi_hat_; }
    const Vec2& eHat() const { return e_hat_; }
    double measuredRange() const { return range_to_target_; }
    bool hasAdapted() const { return adapted_; }

    void setInitialYaw(double psi0) { psi_hat_ = wrapAngle(psi0); }
    void disableExcitation() { c_.exc_amp = 0.0; }

  private:
    struct Sample {
        double t;
        Vec2 bv;
        Vec2 cum_odo;
    };

    ControlParams c_;
    double meas_period_;
    double psi_hat_{0.0};
    Vec2 e_hat_{0.0, 0.0};
    Vec2 cum_odo_{0.0, 0.0};
    double range_to_target_{0.0};
    bool adapted_{false};
    std::deque<Sample> hist_;
};

// ---------------------------------------------------------------------------
// Absolute-state EKF baseline: X = [qx qy px py xx xy psi].
// Sequential scalar updates (valid because measurement noises are mutually
// independent). Without GPS the translation gauge is unobservable and the
// covariance grows along it; with GPS the gauge is anchored at GPS accuracy.
// ---------------------------------------------------------------------------
class FullEkf {
  public:
    static constexpr std::size_t N = 7;

    FullEkf(const SimParams& prm, unsigned seed) : prm_(prm) {
        Rng rng(seed);
        // Coarse initial guesses with large covariance; vehicle assumed to
        // start near the origin of its own control frame.
        X_[0] = 0.0; X_[1] = 0.0;
        X_[2] = rng.uni(-20.0, 20.0); X_[3] = rng.uni(-20.0, 20.0);
        X_[4] = rng.uni(-15.0, 15.0); X_[5] = rng.uni(-15.0, 15.0);
        X_[6] = rng.uni(-kPi, kPi);
        P_ = MatN<N>::identity(1.0);
        P_(2, 2) = P_(3, 3) = 400.0;
        P_(4, 4) = P_(5, 5) = 225.0;
        P_(6, 6) = 10.0;
    }

    void propagate(const Vec2& odo_increment, double dt, double sigma_odo) {
        X_[0] += odo_increment.x;
        X_[1] += odo_increment.y;
        const double qv = sigma_odo * sigma_odo * dt + 1e-8;
        P_(0, 0) += qv; P_(1, 1) += qv;
        P_(6, 6) += 1e-9;
    }

    void updateBeacon(const BeaconMeasurement& m, const NoiseParams& n) {
        scalarUpdate(hRangeVehicle(), m.r_v, n.sigma_range, false);
        scalarUpdate(hBearingVehicle(), m.beta_v, n.sigma_bearing, true);
        scalarUpdate(hRangeTarget(), m.r_t, n.sigma_range, false);
        scalarUpdate(hBearingTarget(), m.beta_t, n.sigma_bearing, true);
    }

    void updateGps(const Vec2& z, double sigma_gps) {
        std::array<double, N> Hx{}; Hx[0] = 1.0;
        scalarUpdateRaw(Hx, z.x - X_[0], sigma_gps);
        std::array<double, N> Hy{}; Hy[1] = 1.0;
        scalarUpdateRaw(Hy, z.y - X_[1], sigma_gps);
    }

    Vec2 qHat() const { return {X_[0], X_[1]}; }
    Vec2 pHat() const { return {X_[2], X_[3]}; }
    Vec2 xHat() const { return {X_[4], X_[5]}; }
    double psiHat() const { return X_[6]; }
    Vec2 relTarget() const { return pHat() - qHat(); }

  private:
    struct Obs {
        std::array<double, N> H{};
        double pred{0.0};
    };

    Obs hRangeVehicle() const {
        Obs o;
        const Vec2 d{X_[0] - X_[4], X_[1] - X_[5]};
        const double r = std::max(d.norm(), 1e-6);
        o.pred = r;
        o.H[0] = d.x / r; o.H[1] = d.y / r;
        o.H[4] = -d.x / r; o.H[5] = -d.y / r;
        return o;
    }

    Obs hBearingVehicle() const {
        Obs o;
        const Vec2 d{X_[0] - X_[4], X_[1] - X_[5]};
        const double r2 = std::max(d.dot(d), 1e-9);
        o.pred = wrapAngle(std::atan2(d.y, d.x) - X_[6]);
        o.H[0] = -d.y / r2; o.H[1] = d.x / r2;
        o.H[4] = d.y / r2; o.H[5] = -d.x / r2;
        o.H[6] = -1.0;
        return o;
    }

    Obs hRangeTarget() const {
        Obs o;
        const Vec2 d{X_[2] - X_[4], X_[3] - X_[5]};
        const double r = std::max(d.norm(), 1e-6);
        o.pred = r;
        o.H[2] = d.x / r; o.H[3] = d.y / r;
        o.H[4] = -d.x / r; o.H[5] = -d.y / r;
        return o;
    }

    Obs hBearingTarget() const {
        Obs o;
        const Vec2 d{X_[2] - X_[4], X_[3] - X_[5]};
        const double r2 = std::max(d.dot(d), 1e-9);
        o.pred = wrapAngle(std::atan2(d.y, d.x) - X_[6]);
        o.H[2] = -d.y / r2; o.H[3] = d.x / r2;
        o.H[4] = d.y / r2; o.H[5] = -d.x / r2;
        o.H[6] = -1.0;
        return o;
    }

    void scalarUpdate(const Obs& o, double z, double sigma, bool angular) {
        double innov = z - o.pred;
        if (angular) innov = wrapAngle(innov);
        scalarUpdateRaw(o.H, innov, sigma);
    }

    void scalarUpdateRaw(const std::array<double, N>& H, double innov, double sigma) {
        std::array<double, N> PH{};
        double S = sigma * sigma;
        for (std::size_t i = 0; i < N; ++i) {
            double s = 0.0;
            for (std::size_t j = 0; j < N; ++j) s += P_(i, j) * H[j];
            PH[i] = s;
        }
        for (std::size_t i = 0; i < N; ++i) S += H[i] * PH[i];
        if (S <= 0.0) return;
        std::array<double, N> K{};
        for (std::size_t i = 0; i < N; ++i) K[i] = PH[i] / S;
        for (std::size_t i = 0; i < N; ++i) X_[i] += K[i] * innov;
        X_[6] = wrapAngle(X_[6]);
        for (std::size_t i = 0; i < N; ++i)
            for (std::size_t j = 0; j < N; ++j)
                P_(i, j) -= K[i] * PH[j];
        // Symmetrize for numerical health.
        for (std::size_t i = 0; i < N; ++i)
            for (std::size_t j = i + 1; j < N; ++j) {
                const double s = 0.5 * (P_(i, j) + P_(j, i));
                P_(i, j) = P_(j, i) = s;
            }
    }

    SimParams prm_;
    std::array<double, N> X_{};
    MatN<N> P_;
};

// ---------------------------------------------------------------------------
// Naive baseline: trust noisy GPS as the true vehicle position and run the
// known-position yaw calibration of the baseline paper on GPS-differenced
// displacements. Shows how GPS noise poisons calibration when it is treated
// as ground truth instead of as one more noisy sensor.
// ---------------------------------------------------------------------------
class NaiveGpsSeeker {
  public:
    NaiveGpsSeeker(const ControlParams& c, double meas_period)
        : c_(c), meas_period_(meas_period) {}

    void update(double t, const BeaconMeasurement& m, const Vec2& q_gps) {
        const Vec2 bv = m.bv();
        const Vec2 d = m.bt() - bv;
        hist_.push_back({t, bv, q_gps});
        while (hist_.size() > 2 && t - hist_.front().t > c_.window) hist_.pop_front();
        const Vec2 dq = q_gps - hist_.front().cum_odo;   // GPS-differenced motion
        if (dq.norm() >= c_.d_min) {
            const Vec2 db = bv - hist_.front().bv;
            const Vec2 res = db - rotateT(psi_hat_, dq);
            const Vec2 grad_dir = rotateT(psi_hat_, Vec2{dq.y, -dq.x});
            psi_hat_ = wrapAngle(psi_hat_ + c_.gamma_psi * meas_period_ *
                                                res.dot(grad_dir) /
                                                (dq.dot(dq) + c_.disp_reg));
        }
        const Vec2 e_meas = rotate(psi_hat_, d);
        const double alpha = 1.0 - std::exp(-c_.k_e * meas_period_);
        e_hat_ += alpha * (e_meas - e_hat_);
    }

    Vec2 control(double t) const {
        Vec2 u = e_hat_ * c_.k_p;
        const double a = c_.exc_amp * std::exp(-c_.exc_lambda * t);
        u += Vec2{a * std::cos(c_.exc_omega * t), a * std::sin(c_.exc_omega * t)};
        return saturate(u, c_.v_max);
    }

    double psiHat() const { return psi_hat_; }
    const Vec2& eHat() const { return e_hat_; }

  private:
    struct Sample {
        double t;
        Vec2 bv;
        Vec2 cum_odo;
    };

    ControlParams c_;
    double meas_period_;
    double psi_hat_{0.0};
    Vec2 e_hat_{0.0, 0.0};
    std::deque<Sample> hist_;
};

}  // namespace gfs

#pragma once
// Fair baseline estimators (plan §4.3).
//
// EkfPipeline: absolute-state SEQUENTIAL-update EKF baseline (one pass of
// scalar updates per packet, no relinearization iterations) in O frame, with
// SAME packet stream, native measurement likelihood, two-view closed-form
// initialization, and a matched constant odometry-bias state. State:
//   X = [s(2), x_O(2), p_O(2), theta(1), b(2)]  (9 states)
// The O-frame gauge is fixed by anchoring the initial pose (Theorem 1).
// It shares the ExcitationSupervisor and seek law so differences measure the
// ESTIMATOR, not the mission logic.

#include <chrono>

#include "Math.hpp"
#include "Supervisor.hpp"
#include "Types.hpp"

namespace gfs {

class EkfPipeline {
  public:
    static constexpr int N = 9;

    EkfPipeline(const SupervisorConfig& scfg) : cfg_(scfg), sup_(scfg) {}

    // Propagate: integrate heading and body increment; after
    // initialization, subtract the body-frame bias state (rotated into O)
    // and inflate covariance with the increment noise.
    void onOdometry(const OdometrySample& o) {
        const double h_mid = heading_O_ + 0.5 * o.dtheta;
        heading_O_ = wrapAngle(heading_O_ + o.dtheta);
        const Vec2 ds = rotate(h_mid, o.dxy);
        s_int_ += ds;
        t_ = o.t;
        if (!initialized_) return;

        const double dt = o.t - last_odo_t_ > 0 ? o.t - last_odo_t_ : 0.005;
        // Propagate: s += ds - R(heading) b * dt. The bias state is in the
        // BODY frame, matching the simulated bias (review #4: the previous
        // odometry-frame bias model was mismatched to the world).
        const double c = std::cos(h_mid), sn = std::sin(h_mid);
        X_[0] += ds.x - (c * X_[7] - sn * X_[8]) * dt;
        X_[1] += ds.y - (sn * X_[7] + c * X_[8]) * dt;
        // Covariance: F = I except ds/db = -R(h_mid)*dt (2x2 block).
        const double f07 = -c * dt, f08 = sn * dt;
        const double f17 = -sn * dt, f18 = -c * dt;
        for (int j = 0; j < N; ++j) {
            const double p7 = P_(7, j), p8 = P_(8, j);
            P_(0, j) += f07 * p7 + f08 * p8;
            P_(1, j) += f17 * p7 + f18 * p8;
        }
        for (int i = 0; i < N; ++i) {
            const double p7 = P_(i, 7), p8 = P_(i, 8);
            P_(i, 0) += f07 * p7 + f08 * p8;
            P_(i, 1) += f17 * p7 + f18 * p8;
        }
        const double q = o.sigma_xy * o.sigma_xy + 1e-10;
        P_(0, 0) += q;
        P_(1, 1) += q;
        P_(6, 6) += 1e-9 * dt;
        P_(7, 7) += 1e-8 * dt;
        P_(8, 8) += 1e-8 * dt;
        last_odo_t_ = o.t;
    }

    // Measurement step: before initialization, wait for two views with a
    // sufficient displacement baseline and initialize via the closed-form
    // two-view construction; afterwards, apply the four native
    // range/bearing scalar updates sequentially. Then run the shared
    // supervisor on the EKF's own certificate (its theta marginal).
    void onPacket(const RelayPacket& pkt) {
        if (!pkt.valid) return;
        const auto t0 = std::chrono::steady_clock::now();
        if (!initialized_) {
            tryInit(pkt);
        } else {
            scalarUpdates(pkt);
        }
        const double rt_us =
            std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - t0)
                .count();

        EstimatorOutput est;
        est.valid = initialized_;
        est.runtime_us = rt_us;
        if (initialized_) {
            est.theta_hat = X_[6];
            est.theta_variance = std::max(P_(6, 6), 1e-12);
            est.e_hat = Vec2{X_[4] - X_[0], X_[5] - X_[1]};
            est.correlation = 1.0;  // not applicable; passes acquisition gate
            est.window_packets = packet_count_;
            est.certified =
                1.96 * std::sqrt(est.theta_variance) <= cfg_.cert_on_rad;
        }
        ++packet_count_;

        bool retrig = false;
        const double dist = initialized_ ? est.e_hat.norm() : 1e9;
        const Mode m = sup_.update(t_, est, dist, &retrig);
        cmd_ = seekCommand(m, est.e_hat, heading_O_, cfg_, retrig, t_);
        last_est_ = est;
    }

    const SeekCommand& command() const { return cmd_; }
    const EstimatorOutput& estimate() const { return last_est_; }
    Vec2 sHat() const {
        return initialized_ ? Vec2{X_[0], X_[1]} : s_int_;
    }
    double headingO() const { return heading_O_; }
    int retriggers() const { return sup_.retriggers(); }
    Mode mode() const { return sup_.mode(); }

  private:
    void tryInit(const RelayPacket& pkt) {
        if (!have_first_) {
            first_s_ = s_int_;
            first_lv_ = pkt.lv();
            have_first_ = true;
            return;
        }
        const Vec2 dq = s_int_ - first_s_;
        if (dq.norm() < 1.0) return;  // wait for a distinct view (tuned: 0.3 m init baseline caused overconfident wrong-yaw locks on 8/200 seeds)
        const Vec2 db = pkt.lv() - first_lv_;
        if (db.norm() < 1e-6) return;
        const double th =
            std::atan2(dq.y, dq.x) - std::atan2(db.y, db.x);  // Theorem 2
        const Vec2 x0 = s_int_ - rotate(th, pkt.lv());
        const Vec2 p0 = x0 + rotate(th, pkt.lt());
        X_ = {s_int_.x, s_int_.y, x0.x, x0.y, p0.x, p0.y, wrapAngle(th), 0, 0};
        P_ = MatN<9>::identity(0.0);
        P_(0, 0) = P_(1, 1) = 0.01;
        P_(2, 2) = P_(3, 3) = 4.0;
        P_(4, 4) = P_(5, 5) = 4.0;
        P_(6, 6) = 0.1;
        P_(7, 7) = P_(8, 8) = 0.05 * 0.05;
        initialized_ = true;
        last_odo_t_ = t_;
    }

    void scalarUpdates(const RelayPacket& pkt) {
        const double sr = std::max(pkt.sigma_r, 1e-6);
        const double sb = std::max(pkt.sigma_beta, 1e-6);
        // Vehicle range/bearing about (s - x).
        updateRange(0, 2, pkt.r_v, sr);
        updateBearing(0, 2, pkt.beta_v, sb);
        // Target range/bearing about (p - x).
        updateRange(4, 2, pkt.r_t, sr);
        updateBearing(4, 2, pkt.beta_t, sb);
    }

    void updateRange(int ia, int ix, double z, double sigma) {
        const Vec2 d{X_[ia] - X_[ix], X_[ia + 1] - X_[ix + 1]};
        const double r = std::max(d.norm(), 1e-6);
        std::array<double, N> H{};
        H[ia] = d.x / r;
        H[ia + 1] = d.y / r;
        H[ix] = -d.x / r;
        H[ix + 1] = -d.y / r;
        scalarUpdate(H, z - r, sigma, false);
    }

    void updateBearing(int ia, int ix, double z, double sigma) {
        const Vec2 d{X_[ia] - X_[ix], X_[ia + 1] - X_[ix + 1]};
        const double r2 = std::max(d.dot(d), 1e-9);
        std::array<double, N> H{};
        H[ia] = -d.y / r2;
        H[ia + 1] = d.x / r2;
        H[ix] = d.y / r2;
        H[ix + 1] = -d.x / r2;
        H[6] = -1.0;
        const double pred = wrapAngle(std::atan2(d.y, d.x) - X_[6]);
        scalarUpdate(H, wrapAngle(z - pred), sigma, true);
    }

    void scalarUpdate(const std::array<double, N>& H, double innov,
                      double sigma, bool /*angular*/) {
        std::array<double, N> PH{};
        double S = sigma * sigma;
        for (int i = 0; i < N; ++i) {
            double s = 0.0;
            for (int j = 0; j < N; ++j) s += P_(i, j) * H[j];
            PH[i] = s;
        }
        for (int i = 0; i < N; ++i) S += H[i] * PH[i];
        if (S <= 0.0) return;
        for (int i = 0; i < N; ++i) X_[i] += (PH[i] / S) * innov;
        X_[6] = wrapAngle(X_[6]);
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) P_(i, j) -= (PH[i] / S) * PH[j];
        for (int i = 0; i < N; ++i)
            for (int j = i + 1; j < N; ++j) {
                const double s = 0.5 * (P_(i, j) + P_(j, i));
                P_(i, j) = P_(j, i) = s;
            }
    }

    SupervisorConfig cfg_;
    ExcitationSupervisor sup_;
    std::array<double, N> X_{};
    MatN<9> P_;
    Vec2 s_int_{0, 0};        // integrated odometry (pre-init reference)
    double heading_O_ = 0.0;
    double t_ = 0.0;
    double last_odo_t_ = 0.0;
    bool initialized_ = false;
    bool have_first_ = false;
    Vec2 first_s_, first_lv_;
    int packet_count_ = 0;
    SeekCommand cmd_;
    EstimatorOutput last_est_;
};

}  // namespace gfs

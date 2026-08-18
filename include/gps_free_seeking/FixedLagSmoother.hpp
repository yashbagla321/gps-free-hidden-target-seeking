#pragma once
// FixedLagSmoother (plan §4.1 component 4).
//
// Damped Gauss-Newton over a sliding window of packet-time vehicle poses with
// native range-bearing residuals (wrapped bearings), odometry factors, a
// constant odometry-bias state, and Huber loss on relay residuals.
//
// State layout: z = [x_O(2), p_O(2), theta(1), b(2), s_1..s_{K-1} (2K-2)];
// the first pose s_0 is gauge-fixed to its odometry-integrated value
// (Theorem 1: the odometry gauge must be fixed, not estimated).
//
// Output: refined theta and its marginal variance. The caller applies the
// acceptance gate (amendment 1): smoother output is used only when its
// variance beats the registration certificate; the certified registration
// path never depends on this component.
//
// Prior carryover: after each solve, the marginal information of the head
// block [x_O, p_O, theta, b] is retained and re-applied as a Gaussian prior
// when the window slides (pose cross-terms dropped -- documented
// approximation; PSD verified by acceptance test).

#include <deque>
#include <vector>

#include "LinAlg.hpp"
#include "Math.hpp"
#include "Types.hpp"

namespace gfs {

struct SmootherConfig {
    int max_poses = 32;
    int max_iters = 15;
    double huber_delta = 2.0;      // in whitened-residual units
    double lambda0 = 1e-3;         // initial LM damping
    double min_range = 1e-2;       // guard
    double prior_bias_sigma = 0.05;  // [m/s] initial prior on odometry bias
    // Head-prior carryover across solves. DISABLED by default: consecutive
    // solves share most of their window, so re-applying the previous head
    // marginal double-counts information, collapses the reported variance,
    // and (via the best-ever control-path gate) locks in early yaw errors --
    // observed as smoother-variant regressions in the S3 smoke run
    // (2026-08-16). Re-enable only with proper drop-node marginalization.
    bool prior_carryover = false;
};

class FixedLagSmoother {
  public:
    explicit FixedLagSmoother(const SmootherConfig& cfg = {}) : cfg_(cfg) {}

    struct Node {
        double t = 0.0;
        Vec2 s_init;         // odometry-integrated pose at packet time
        RelayPacket pkt;
        Vec2 odo_delta;      // integrated odometry delta from previous node
        double odo_sigma = 1e-3;  // std of that delta per axis [m]
        double dt_prev = 0.0;     // time since previous node [s]
    };

    void addNode(const Node& n) {
        nodes_.push_back(n);
        while (static_cast<int>(nodes_.size()) > cfg_.max_poses) {
            nodes_.pop_front();
            // Window slid: previous head prior remains valid as a prior on
            // [x_O, p_O, theta, b] (static states), poses re-enter fresh.
        }
    }

    int size() const { return static_cast<int>(nodes_.size()); }

    // Initialize head states from a registration solution.
    void setInitialGuess(double theta, const Vec2& x_o, const Vec2& p_o) {
        theta_ = theta;
        x_o_ = x_o;
        p_o_ = p_o;
        have_guess_ = true;
    }

    // Damped Gauss-Newton over the window.
    // Algorithm: initialize head states from the registration guess and
    // poses from integrated odometry; iterate build -> LM-damped normal
    // solve -> accept on cost decrease; on convergence, extract the head
    // marginal covariance by solving against unit vectors, which supplies
    // thetaVariance() for the caller's acceptance gate.
    bool solve() {
        const int K = size();
        if (K < 3 || !have_guess_) return false;
        const int n = 7 + 2 * (K - 1);  // head + poses s_1..s_{K-1}

        // Working state.
        std::vector<double> z(n, 0.0);
        z[0] = x_o_.x; z[1] = x_o_.y;
        z[2] = p_o_.x; z[3] = p_o_.y;
        z[4] = theta_;
        z[5] = bias_.x; z[6] = bias_.y;
        for (int k = 1; k < K; ++k) {
            z[7 + 2 * (k - 1)] = nodes_[k].s_init.x;
            z[7 + 2 * (k - 1) + 1] = nodes_[k].s_init.y;
        }

        double lambda = cfg_.lambda0;
        double prev_cost = buildAndCost(z, nullptr, nullptr);
        for (int it = 0; it < cfg_.max_iters; ++it) {
            DenseMat H(n, n);
            std::vector<double> g(n, 0.0);
            buildAndCost(z, &H, &g);
            // LM damping.
            DenseMat Hd = H;
            for (int i = 0; i < n; ++i) Hd(i, i) += lambda * (1.0 + H(i, i));
            std::vector<double> dz;
            if (!choleskySolve(Hd, g, &dz)) {
                lambda *= 10.0;
                continue;
            }
            std::vector<double> z_try = z;
            for (int i = 0; i < n; ++i) z_try[i] -= dz[i];
            z_try[4] = wrapAngle(z_try[4]);
            const double cost = buildAndCost(z_try, nullptr, nullptr);
            if (cost < prev_cost) {
                z = z_try;
                prev_cost = cost;
                lambda = std::max(lambda * 0.3, 1e-9);
                double step2 = 0.0;
                for (double d : dz) step2 += d * d;
                if (step2 < 1e-14) break;
            } else {
                lambda *= 10.0;
                if (lambda > 1e6) break;
            }
        }

        // Commit and compute the head marginal covariance.
        DenseMat H(n, n);
        std::vector<double> g(n, 0.0);
        buildAndCost(z, &H, &g);
        if (!headMarginal(H)) return false;

        x_o_ = Vec2{z[0], z[1]};
        p_o_ = Vec2{z[2], z[3]};
        theta_ = z[4];
        bias_ = Vec2{z[5], z[6]};
        last_pose_ = (K >= 2) ? Vec2{z[7 + 2 * (K - 2)], z[7 + 2 * (K - 2) + 1]}
                              : nodes_.front().s_init;
        last_pose_t_ = nodes_.back().t;
        // Head prior for the next solve (only when carryover is enabled).
        prior_mean_ = {z[0], z[1], z[2], z[3], z[4], z[5], z[6]};
        have_prior_ = cfg_.prior_carryover;
        solved_ = true;
        return true;
    }

    double theta() const { return theta_; }
    double thetaVariance() const { return theta_var_; }
    // Smoothed estimate of the newest window pose (for path-error metrics;
    // P0 finding: evaluating the smoother's path with raw integrated
    // odometry measured nothing).
    const Vec2& lastPose() const { return last_pose_; }
    double lastPoseTime() const { return last_pose_t_; }
    const Vec2& xO() const { return x_o_; }
    const Vec2& pO() const { return p_o_; }
    const Vec2& bias() const { return bias_; }
    bool solvedOnce() const { return solved_; }
    const DenseMat& headInformation() const { return prior_info_; }

  private:
    // Build normal equations (H, g) and return the robust cost. When H/g are
    // null, only the cost is computed (for LM step acceptance).
    double buildAndCost(const std::vector<double>& z, DenseMat* H,
                        std::vector<double>* g) {
        const int K = size();
        double cost = 0.0;

        auto poseOf = [&](int k) -> Vec2 {
            if (k == 0) return nodes_[0].s_init;  // gauge-fixed
            return Vec2{z[7 + 2 * (k - 1)], z[7 + 2 * (k - 1) + 1]};
        };
        auto poseIdx = [&](int k) -> int { return 7 + 2 * (k - 1); };

        const double th = z[4];
        const Vec2 xo{z[0], z[1]};
        const Vec2 po{z[2], z[3]};
        const Vec2 b{z[5], z[6]};

        // Accumulator for one scalar residual row.
        auto addRow = [&](const std::vector<std::pair<int, double>>& jac,
                          double r, double w_huber) {
            cost += 0.5 * w_huber * r * r;
            if (!H) return;
            for (const auto& [i, ji] : jac) {
                (*g)[i] += w_huber * ji * r;
                for (const auto& [j, jj] : jac) (*H)(i, j) += w_huber * ji * jj;
            }
        };
        auto huberW = [&](double r) {
            const double a = std::fabs(r);
            return a <= cfg_.huber_delta ? 1.0 : cfg_.huber_delta / a;
        };

        for (int k = 0; k < K; ++k) {
            const auto& nd = nodes_[k];
            const Vec2 sk = poseOf(k);

            const double sig_r = std::max(nd.pkt.sigma_r, 1e-6);
            const double sig_b = std::max(nd.pkt.sigma_beta, 1e-6);
            // Vehicle range: (|sk - xo| - r_v)/sigma_r.
            {
                Vec2 d = sk - xo;
                const double rho = std::max(d.norm(), cfg_.min_range);
                const double inv_s = 1.0 / sig_r;
                const double r = (rho - nd.pkt.r_v) * inv_s;
                const double w = huberW(r);
                std::vector<std::pair<int, double>> jac;
                const Vec2 u = d * (1.0 / rho);
                if (k > 0) {
                    jac.push_back({poseIdx(k), u.x * inv_s});
                    jac.push_back({poseIdx(k) + 1, u.y * inv_s});
                }
                jac.push_back({0, -u.x * inv_s});
                jac.push_back({1, -u.y * inv_s});
                addRow(jac, r, w);
            }
            // Vehicle bearing: wrap(angle(sk - xo) - theta - beta_v)/sigma_b.
            {
                Vec2 d = sk - xo;
                const double r2 = std::max(d.dot(d), cfg_.min_range * cfg_.min_range);
                const double inv_s = 1.0 / sig_b;
                const double r =
                    wrapAngle(std::atan2(d.y, d.x) - th - nd.pkt.beta_v) * inv_s;
                const double w = huberW(r);
                std::vector<std::pair<int, double>> jac;
                const double jx = -d.y / r2, jy = d.x / r2;
                if (k > 0) {
                    jac.push_back({poseIdx(k), jx * inv_s});
                    jac.push_back({poseIdx(k) + 1, jy * inv_s});
                }
                jac.push_back({0, -jx * inv_s});
                jac.push_back({1, -jy * inv_s});
                jac.push_back({4, -1.0 * inv_s});
                addRow(jac, r, w);
            }
            // Target range and bearing (target static: same p_o each packet).
            {
                Vec2 d = po - xo;
                const double rho = std::max(d.norm(), cfg_.min_range);
                const double inv_s = 1.0 / sig_r;
                const double r = (rho - nd.pkt.r_t) * inv_s;
                const double w = huberW(r);
                const Vec2 u = d * (1.0 / rho);
                addRow({{2, u.x * inv_s}, {3, u.y * inv_s},
                        {0, -u.x * inv_s}, {1, -u.y * inv_s}}, r, w);
            }
            {
                Vec2 d = po - xo;
                const double r2 = std::max(d.dot(d), cfg_.min_range * cfg_.min_range);
                const double inv_s = 1.0 / sig_b;
                const double r =
                    wrapAngle(std::atan2(d.y, d.x) - th - nd.pkt.beta_t) * inv_s;
                const double w = huberW(r);
                const double jx = -d.y / r2, jy = d.x / r2;
                addRow({{2, jx * inv_s}, {3, jy * inv_s},
                        {0, -jx * inv_s}, {1, -jy * inv_s},
                        {4, -1.0 * inv_s}}, r, w);
            }
            // Odometry factor between consecutive poses:
            // r = (s_k - s_{k-1}) - (odo_delta - b*dt), quadratic (no Huber).
            if (k > 0) {
                const Vec2 sp = poseOf(k - 1);
                const double inv_s = 1.0 / std::max(nd.odo_sigma, 1e-6);
                const double dt = nd.dt_prev;
                for (int ax = 0; ax < 2; ++ax) {
                    const double meas =
                        (ax == 0 ? nd.odo_delta.x : nd.odo_delta.y);
                    const double sk_ax = (ax == 0 ? sk.x : sk.y);
                    const double sp_ax = (ax == 0 ? sp.x : sp.y);
                    const double b_ax = (ax == 0 ? b.x : b.y);
                    const double r = (sk_ax - sp_ax - meas + b_ax * dt) * inv_s;
                    std::vector<std::pair<int, double>> jac;
                    jac.push_back({poseIdx(k) + ax, inv_s});
                    if (k - 1 > 0) jac.push_back({poseIdx(k - 1) + ax, -inv_s});
                    jac.push_back({5 + ax, dt * inv_s});
                    addRow(jac, r, 1.0);
                }
            }
        }
        // Bias prior (weak) + carried-over head prior.
        for (int ax = 0; ax < 2; ++ax) {
            const double inv_s = 1.0 / cfg_.prior_bias_sigma;
            addRow({{5 + ax, inv_s}}, z[5 + ax] * inv_s, 1.0);
        }
        if (have_prior_ && H) {
            for (int i = 0; i < 7; ++i)
                for (int j = 0; j < 7; ++j) (*H)(i, j) += prior_info_(i, j);
            for (int i = 0; i < 7; ++i) {
                double gi = 0.0;
                for (int j = 0; j < 7; ++j)
                    gi += prior_info_(i, j) * (z[j] - prior_mean_[j]);
                (*g)[i] += gi;
            }
        }
        if (have_prior_) {
            for (int i = 0; i < 7; ++i) {
                double q = 0.0;
                for (int j = 0; j < 7; ++j)
                    q += (z[i] - prior_mean_[i]) * prior_info_(i, j) *
                         (z[j] - prior_mean_[j]);
                cost += 0.5 * q;
            }
        }
        return cost;
    }

    // Head marginal covariance: solve H X = E for the 7 head unit vectors.
    bool headMarginal(const DenseMat& H) {
        const int n = H.rows;
        DenseMat Hd = H;
        for (int i = 0; i < n; ++i) Hd(i, i) += 1e-9;
        DenseMat cov(7, 7);
        std::vector<double> e(n, 0.0), col;
        for (int j = 0; j < 7; ++j) {
            e.assign(n, 0.0);
            e[j] = 1.0;
            if (!choleskySolve(Hd, e, &col)) return false;
            for (int i = 0; i < 7; ++i) cov(i, j) = col[i];
        }
        for (int i = 0; i < 7; ++i)
            for (int j = i + 1; j < 7; ++j) {
                const double s = 0.5 * (cov(i, j) + cov(j, i));
                cov(i, j) = cov(j, i) = s;
            }
        theta_var_ = std::max(cov(4, 4), 1e-12);
        // Prior information for carryover = inverse of head covariance.
        DenseMat info;
        if (!spdInverse(cov, &info)) return false;
        prior_info_ = info;
        return true;
    }

    SmootherConfig cfg_;
    std::deque<Node> nodes_;
    Vec2 x_o_, p_o_, bias_{0, 0};
    Vec2 last_pose_{0, 0};
    double last_pose_t_ = -1.0;
    double theta_ = 0.0;
    double theta_var_ = 1e9;
    bool have_guess_ = false;
    bool solved_ = false;
    bool have_prior_ = false;
    DenseMat prior_info_{7, 7};
    std::array<double, 7> prior_mean_{};
};

}  // namespace gfs

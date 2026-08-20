#pragma once
// WeightedWindowRegistration (plan §4.1 component 1).
//
// Estimates the relay-to-odometry yaw theta from a sliding window of
// (odometry pose s_k, relay-frame vehicle vector l_k^v) pairs:
//
//   s_k = x_O + R(theta) l_k^v  (+ noise)
//
// theta_hat: scalar-weighted centered registration (closed form; the
//   two-view initializer of Theorem 2 is the K=2 special case).
// Information diagnostic: Schur-complement yaw information I_theta from the
//   3x3 (x_O, theta) conditional information matrix with per-packet
//   Cartesianized anisotropic covariance. It is exact for known poses; with
//   uncertain odometry poses it uses marginal inflation and is not the
//   supervisor certificate. S_v is the interpretable geometry statistic.
//
// The window treats s_k as known; odometry error inside the window is the
// bounded disturbance handled by Proposition 2 (finite-window drift bound).

#include <chrono>
#include <deque>
#include <vector>

#include "LinAlg.hpp"
#include "Math.hpp"
#include "Types.hpp"

namespace gfs {

// Covariance-ablation switch (review response, Phase 3): isolates the
// contribution of modeling cross-view odometry correlation in closed form.
// kFull is the paper's certificate (Theorem 3): the exact O(K) suffix-sum
// treatment of correlated integrated-translation pose errors. kDiag drops
// the cross-view correlation and treats each view's pose error as
// independent (the naive covariance a non-gauge-aware estimator would use);
// kPacketOnly drops the pose term entirely, certifying only packet noise as
// if odometry were exact. Both ablations are expected to undercover under
// correlated odometry -- that gap is the experiment.
enum class VarianceModel { kFull = 0, kDiag = 1, kPacketOnly = 2 };

struct RegistrationConfig {
    int max_packets = 64;      // window capacity
    double max_age = 4.0;      // [s] window horizon
    int min_packets = 2;       // minimum views for a valid estimate
    double min_spread = 1e-4;  // [m^2] guard against repeated-pose windows
    bool endpoint_only = false;  // ablation: use only the two window endpoints
                                 // (closed-form equivalent of the legacy
                                 // endpoint-gradient estimator)
    VarianceModel variance_model = VarianceModel::kFull;
};

class WeightedWindowRegistration {
  public:
    explicit WeightedWindowRegistration(const RegistrationConfig& cfg = {})
        : cfg_(cfg) {}

    void addView(double t, const Vec2& s, const RelayPacket& pkt,
                 double pose_var = 0.0) {
        Entry e;
        e.t = t;
        e.s = s;
        e.pose_var = pose_var;
        e.lv = pkt.lv();
        e.r = std::max(pkt.r_v, 1e-3);
        e.sigma_r = std::max(pkt.sigma_r, 1e-6);
        e.sigma_t = std::max(e.r * pkt.sigma_beta, 1e-6);  // tangential std
        // Insert sorted by measurement timestamp: delayed/jittered delivery
        // can be out of order, and the window logic (front=oldest,
        // endpoint ablation, age pruning) requires time order.
        auto it = win_.end();
        while (it != win_.begin() && (it - 1)->t > e.t) --it;
        win_.insert(it, e);
        while (static_cast<int>(win_.size()) > cfg_.max_packets) win_.pop_front();
        const double t_new = win_.back().t;
        while (!win_.empty() && t_new - win_.front().t > cfg_.max_age)
            win_.pop_front();
    }

    void clearStale(double t) {
        while (!win_.empty() && t - win_.front().t > cfg_.max_age) win_.pop_front();
    }

    int windowSize() const { return static_cast<int>(win_.size()); }

    // Solve for theta on the current window. Returns false when the window is
    // insufficient (too few views or repeated-pose degeneracy, Corollary
    // "repeated-pose gauge").
    //
    // Outputs two distinct uncertainty quantities (execution-log decision
    // 2026-08-16): info_theta is the Schur-complement information BOUND
    // (Lemma 1); var_theta is the first-order variance actually DELIVERED by
    // the scalar-weighted registration estimator,
    //   var(theta_hat) = sum w_k^2 ||b_k||^2 sigma_perp,k^2 / W^2,
    //   W = sum w_k b_k . a_k magnitude,
    // which is what the supervisor certificate must use. For integrated
    // translation odometry, var(theta_hat) also includes the exact
    // first-order contribution of the correlated random-walk pose errors,
    // Cov(e_i,e_j)=min(v_i,v_j)-v_min. The information bound is reported
    // separately, and the fixed-lag smoother is evaluated as an optional
    // refinement baseline.
    bool solve(double* theta_hat, double* var_theta, double* info_theta,
               double* spread, double* correlation) {
        const auto t0 = std::chrono::steady_clock::now();
        const int n = windowSize();
        if (n < cfg_.min_packets) return false;

        // Entry set: full window, or the two endpoints for the legacy-style
        // ablation (closed-form equivalent of the endpoint-gradient method).
        std::vector<const Entry*> use;
        if (cfg_.endpoint_only) {
            use = {&win_.front(), &win_.back()};
        } else {
            for (const auto& e : win_) use.push_back(&e);
        }

        // Certificate honesty under odometry noise (2026-08-16 review
        // follow-up): window poses are odometry-integrated, not exact. Each
        // view's effective noise is inflated by the pose variance accumulated
        // relative to the window's OLDEST view (common translation cancels in
        // centering; the relative part corrupts the rotation estimate).
        double min_pv = 1e18;
        for (const auto* pe : use) min_pv = std::min(min_pv, pe->pose_var);
        auto effR2 = [&](const Entry& e) {
            return e.sigma_r * e.sigma_r + (e.pose_var - min_pv);
        };
        auto effT2 = [&](const Entry& e) {
            return e.sigma_t * e.sigma_t + (e.pose_var - min_pv);
        };

        // Scalar weights from per-packet isotropicized variance.
        double sw = 0.0;
        Vec2 ms{0, 0}, ml{0, 0};
        for (const auto* pe : use) {
            const auto& e = *pe;
            const double w = 2.0 / (effR2(e) + effT2(e));
            sw += w;
            ms += e.s * w;
            ml += e.lv * w;
        }
        ms = ms * (1.0 / sw);
        ml = ml * (1.0 / sw);

        double cx = 0.0, cy = 0.0, s_v = 0.0, denom = 0.0;
        for (const auto* pe : use) {
            const auto& e = *pe;
            const double w = 2.0 / (effR2(e) + effT2(e));
            const Vec2 a = e.s - ms;   // centered odometry
            const Vec2 b = e.lv - ml;  // centered relay-frame vehicle vector
            cx += w * b.dot(a);        // sum w b . a
            cy += w * b.cross(a);      // sum w b x a
            s_v += b.dot(b);           // unweighted geometry statistic
            denom += w * a.norm() * b.norm();
        }
        *spread = s_v;
        if (s_v < cfg_.min_spread || denom <= 0.0) return false;

        const double th = std::atan2(cy, cx);
        *theta_hat = th;
        *correlation = std::sqrt(cx * cx + cy * cy) / denom;  // in [0, 1]

        // First-order variance of the implemented estimator. Relay-packet
        // noise is independent across views; integrated translation-odometry
        // errors are correlated across views because increments are shared.
        // d theta_hat = sum w_k (nu_k x b_k) / W with W = sum w_k ||b_k||^2;
        // E[(nu x b)^2] = ||b||^2 * (perp-to-b variance of the packet noise).
        {
            const double W = std::sqrt(cx * cx + cy * cy);
            double sensor_num = 0.0;
            std::vector<Vec2> pose_sensitivity;
            pose_sensitivity.reserve(use.size());
            const double C2 = std::max(cx * cx + cy * cy, 1e-24);
            for (const auto* pe : use) {
                const auto& e = *pe;
                const double w = 2.0 / (effR2(e) + effT2(e));
                const Vec2 b = e.lv - ml;
                const double bn = std::max(b.norm(), 1e-9);
                const Vec2 bperp{-b.y / bn, b.x / bn};
                const Vec2 ur = e.lv * (1.0 / std::max(e.lv.norm(), 1e-9));
                const Vec2 ut{-ur.y, ur.x};
                const double sperp2 =
                    e.sigma_r * e.sigma_r * ur.dot(bperp) * ur.dot(bperp) +
                    e.sigma_t * e.sigma_t * ut.dot(bperp) * ut.dot(bperp);
                sensor_num += w * w * bn * bn * sperp2;

                // d theta / d e_k for a perturbation e_k of odometry pose s_k.
                pose_sensitivity.push_back(Vec2{
                    w * (-cx * b.y - cy * b.x) / C2,
                    w * (cx * b.x - cy * b.y) / C2});
            }
            // O(K) evaluation of g^T Cov(e) g. A random-walk increment
            // between views m-1 and m appears in every later pose, so its
            // sensitivity is the suffix sum of the per-pose sensitivities
            // (kFull, Theorem 3). The ablations below are deliberately
            // wrong models of the same pose-error term, kept for the
            // covariance-ablation study (Phase 3 review response): kDiag
            // treats each view's pose error as independent of the others
            // (drops the shared-increment correlation the suffix sum
            // captures); kPacketOnly drops the pose term altogether.
            double pose_var = 0.0;
            switch (cfg_.variance_model) {
                case VarianceModel::kPacketOnly:
                    break;
                case VarianceModel::kDiag:
                    for (size_t k = 0; k < use.size(); ++k) {
                        const double dv =
                            std::max(0.0, use[k]->pose_var - min_pv);
                        pose_var +=
                            dv * pose_sensitivity[k].dot(pose_sensitivity[k]);
                    }
                    break;
                case VarianceModel::kFull:
                default: {
                    Vec2 suffix{0, 0};
                    for (size_t m = use.size() - 1; m > 0; --m) {
                        suffix += pose_sensitivity[m];
                        const double dv = std::max(
                            0.0, use[m]->pose_var - use[m - 1]->pose_var);
                        pose_var += dv * suffix.dot(suffix);
                    }
                    break;
                }
            }
            *var_theta =
                (W > 1e-12) ? sensor_num / (W * W) + pose_var : 1e9;
        }

        // Conditional anisotropic information for xi = (x_O, theta). This is
        // exact when s_k is known. With uncertain integrated odometry, effR2
        // and effT2 provide a marginal diagnostic, while var_theta above is
        // the deployable correlated random-walk certificate.
        //   residual rho_k = s_k - x_O - R(theta) l_k^v,
        //   Sigma_k^O = R(theta) G_k diag(sigma_r^2, sigma_t^2) G_k^T R(theta)^T,
        //   H_k = [ -I, -S R(theta) l_k^v ].
        DenseMat info(3, 3);
        for (const auto* pe : use) {
            const auto& e = *pe;
            // Radial/tangential unit frame of l_k^v in B.
            const Vec2 u = e.lv * (1.0 / std::max(e.lv.norm(), 1e-9));
            const Vec2 up{-u.y, u.x};
            // Sigma^B = G D G^T; its inverse = G D^{-1} G^T.
            const double ir = 1.0 / effR2(e);
            const double it = 1.0 / effT2(e);
            // W^B = ir * u u^T + it * up up^T (2x2), rotate into O:
            // W^O = R(th) W^B R(th)^T  (since Sigma rotates, inverse rotates).
            const Vec2 ru = rotate(th, u);
            const Vec2 rup = rotate(th, up);
            const double W00 = ir * ru.x * ru.x + it * rup.x * rup.x;
            const double W01 = ir * ru.x * ru.y + it * rup.x * rup.y;
            const double W11 = ir * ru.y * ru.y + it * rup.y * rup.y;
            // Jacobian columns: J_x = -I (2x2), J_th = -S R(th) l_k^v (2x1).
            const Vec2 rl = rotate(th, e.lv);
            const Vec2 jth{-(-rl.y), -(rl.x)};  // -S*rl with S=[[0,-1],[1,0]]
            // Accumulate H^T W H over [x(2), theta(1)].
            // x-x block: (+I) W (+I) = W (sign cancels).
            info(0, 0) += W00;
            info(0, 1) += W01;
            info(1, 0) += W01;
            info(1, 1) += W11;
            // x-theta block: (-I)^T W (jth) = W * jth (times (-1)*(-1)=+1
            // because both Jacobian blocks carry the minus sign).
            const double wx = W00 * jth.x + W01 * jth.y;
            const double wy = W01 * jth.x + W11 * jth.y;
            info(0, 2) += wx;
            info(2, 0) += wx;
            info(1, 2) += wy;
            info(2, 1) += wy;
            // theta-theta.
            info(2, 2) += jth.x * wx + jth.y * wy;
        }
        // Schur complement: I_theta = I_tt - I_tx I_xx^{-1} I_xt.
        const double det = info(0, 0) * info(1, 1) - info(0, 1) * info(0, 1);
        if (det <= 1e-12) return false;
        const double inv00 = info(1, 1) / det;
        const double inv01 = -info(0, 1) / det;
        const double inv11 = info(0, 0) / det;
        const double bx = info(0, 2), by = info(1, 2);
        const double schur = info(2, 2) -
                             (bx * (inv00 * bx + inv01 * by) +
                              by * (inv01 * bx + inv11 * by));
        *info_theta = std::max(schur, 0.0);
        last_runtime_us_ =
            std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - t0)
                .count();
        return *info_theta > 0.0;
    }

    double lastRuntimeUs() const { return last_runtime_us_; }

    struct Entry {
        double t;
        Vec2 s;       // odometry pose at packet timestamp (interpolated)
        Vec2 lv;      // relay-frame vehicle vector
        double r;     // measured range (for tangential noise scaling)
        double sigma_r;
        double sigma_t;
        double pose_var = 0.0;  // accumulated odometry variance at this view
    };
    const std::deque<Entry>& window() const { return win_; }

  private:
    RegistrationConfig cfg_;
    std::deque<Entry> win_;
    double last_runtime_us_ = 0.0;
};

}  // namespace gfs

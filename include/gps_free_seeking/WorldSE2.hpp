#pragma once
// Ground-truth world for the Campaign 2027 stack (plan §3.1, §5, §6).
//
// Truth lives in W. The vehicle is a unicycle with commanded (v, omega).
// The odometry frame O has unknown placement: O's origin is the vehicle start
// q0, O's orientation offset is gamma (drawn per trial). The estimator only
// ever sees body-frame OdometrySamples and RelayPackets; the evaluator alone
// may query truth.
//
// Degradation models (plan §5 studies 6-8): odometry translation noise,
// constant velocity bias, scale error, heading drift; relay packet noise,
// Bernoulli dropout, fixed+jitter delay, bearing/range outliers.

#include <deque>

#include "Math.hpp"
#include "Types.hpp"

namespace gfs {

struct OdomErrorParams {
    double sigma_xy = 0.005;       // per-increment translation noise [m] at dt
    double sigma_dtheta = 0.001;   // per-increment heading noise [rad] at dt
    Vec2 bias_vel{0.0, 0.0};       // body-frame velocity bias [m/s]
    double scale_error = 0.0;      // multiplicative on translation (e.g. 0.08)
    double heading_drift = 0.0;    // deterministic drift rate [rad/s]
};

struct RelayErrorParams {
    double sigma_r = 0.10;         // [m]
    double sigma_beta = 0.0175;    // [rad] (~1 deg)
    double dropout = 0.0;          // Bernoulli drop probability
    double delay = 0.0;            // fixed delivery delay [s]
    double delay_jitter = 0.0;     // uniform +/- jitter on delay [s]
    double outlier_prob = 0.0;     // per-packet gross error probability
    double outlier_scale = 10.0;   // outlier magnitude in sigmas
};

struct TrialGeometry {
    Vec2 q0;            // vehicle start in W
    double heading0;    // vehicle start heading in W
    Vec2 target;        // p in W
    Vec2 relay;         // x in W
    double relay_yaw;   // psi in W
    double gamma;       // O-frame orientation offset in W
};

// Randomized admissible geometry per plan §5: target range 6-15 m, relay
// range 4-12 m, all angles uniform, 1 m minimum separation.
inline TrialGeometry sampleGeometry(Rng& rng) {
    TrialGeometry g;
    g.q0 = Vec2{0.0, 0.0};
    g.heading0 = rng.uni(-kPi, kPi);
    g.gamma = rng.uni(-kPi, kPi);
    g.relay_yaw = rng.uni(-kPi, kPi);
    for (;;) {
        const double at = rng.uni(-kPi, kPi);
        const double rt = rng.uni(6.0, 15.0);
        const double ax = rng.uni(-kPi, kPi);
        const double rx = rng.uni(4.0, 12.0);
        g.target = Vec2{rt * std::cos(at), rt * std::sin(at)};
        g.relay = Vec2{rx * std::cos(ax), rx * std::sin(ax)};
        if ((g.target - g.relay).norm() >= 1.0 && (g.relay - g.q0).norm() >= 1.0)
            return g;
    }
}

class WorldSE2 {
  public:
    WorldSE2(const TrialGeometry& geo, const OdomErrorParams& oe,
             const RelayErrorParams& re, double dt, unsigned seed)
        : geo_(geo), oe_(oe), re_(re), dt_(dt), rng_(seed),
          q_(geo.q0), heading_(geo.heading0) {}

    // Advance truth with a unicycle command; returns the corrupted body-frame
    // odometry increment the vehicle receives.
    OdometrySample step(const SeekCommand& cmd, double t) {
        const double dth = cmd.omega * dt_;
        // Truth integration (midpoint heading for the arc).
        const double hmid = heading_ + 0.5 * dth;
        q_ += Vec2{cmd.v * dt_ * std::cos(hmid), cmd.v * dt_ * std::sin(hmid)};
        heading_ = wrapAngle(heading_ + dth);

        OdometrySample o;
        o.t = t + dt_;
        const double s = 1.0 + oe_.scale_error;
        o.dxy = Vec2{cmd.v * dt_ * s + oe_.bias_vel.x * dt_ + rng_.gauss(oe_.sigma_xy),
                     oe_.bias_vel.y * dt_ + rng_.gauss(oe_.sigma_xy)};
        o.dtheta = dth + oe_.heading_drift * dt_ + rng_.gauss(oe_.sigma_dtheta);
        o.sigma_xy = oe_.sigma_xy;
        o.sigma_dtheta = oe_.sigma_dtheta;
        return o;
    }

    // Generate a relay packet for measurement time t; may be dropped or
    // delivered late. Callers poll deliverDue() for arrivals.
    void emitPacket(double t) {
        if (re_.dropout > 0.0 && rng_.uni(0.0, 1.0) < re_.dropout) return;
        RelayPacket m;
        m.t = t;
        // Optional mid-mission relay yaw step (supervisor recovery study).
        const double psi = (yaw_step_time_ >= 0.0 && t >= yaw_step_time_)
                               ? geo_.relay_yaw + yaw_step_size_
                               : geo_.relay_yaw;
        // One outlier event PER PACKET (e.g. NLOS multipath corrupts the
        // whole return), not per channel.
        const bool outlier =
            re_.outlier_prob > 0.0 && rng_.uni(0.0, 1.0) < re_.outlier_prob;
        const Vec2 tgt = currentTarget(t);
        const Vec2 rel_v = rotateT(psi, q_ - geo_.relay);
        const Vec2 rel_t = rotateT(psi, tgt - geo_.relay);
        m.r_v = rel_v.norm() + noiseCh(re_.sigma_r, outlier);
        m.beta_v = std::atan2(rel_v.y, rel_v.x) + noiseCh(re_.sigma_beta, outlier);
        m.r_t = rel_t.norm() + noiseCh(re_.sigma_r, outlier);
        m.beta_t = std::atan2(rel_t.y, rel_t.x) + noiseCh(re_.sigma_beta, outlier);
        m.sigma_r = re_.sigma_r;
        m.sigma_beta = re_.sigma_beta;
        m.valid = true;
        double delay = re_.delay;
        if (re_.delay_jitter > 0.0)
            delay += rng_.uni(-re_.delay_jitter, re_.delay_jitter);
        queue_.push_back({t + std::max(0.0, delay), m});
    }

    // Pop packets whose delivery time has arrived (in-order timestamps kept;
    // late arrivals keep their original measurement timestamp m.t).
    bool deliverDue(double now, RelayPacket* out) {
        for (auto it = queue_.begin(); it != queue_.end(); ++it) {
            if (it->deliver_at <= now) {
                *out = it->pkt;
                queue_.erase(it);
                return true;
            }
        }
        return false;
    }

    // ---- Evaluator-only truth access (never fed to the estimator) ---------
    // Frame convention: the vehicle-side pipeline integrates odometry from
    // s = 0, heading = 0, so ITS odometry frame O has origin q0 and
    // orientation equal to the initial vehicle heading. All truth-in-O
    // quantities below use that convention (gamma in TrialGeometry remains a
    // free parameter of the world; it must not appear in any vehicle-side or
    // evaluator-side quantity -- that is Theorem 1 at work).
    const Vec2& q() const { return q_; }
    double heading() const { return heading_; }
    double distToTarget() const { return (geo_.target - q_).norm(); }
    double distToTargetAt(double t) const {
        return (currentTarget(t) - q_).norm();
    }
    Vec2 currentTarget(double t) const {
        return (target_step_time_ >= 0.0 && t >= target_step_time_)
                   ? geo_.target + target_offset_
                   : geo_.target;
    }
    double trueTheta() const {
        return wrapAngle(geo_.relay_yaw - geo_.heading0);
    }
    // Truth vehicle position in O (for path RMSE evaluation).
    Vec2 sTrue() const { return rotateT(geo_.heading0, q_ - geo_.q0); }
    double headingO() const { return wrapAngle(heading_ - geo_.heading0); }
    Vec2 targetO() const {
        return rotateT(geo_.heading0, geo_.target - geo_.q0);
    }
    const TrialGeometry& geometry() const { return geo_; }

    // Configure a relay yaw step at time t (change-detection study).
    void setYawStep(double t, double delta) {
        yaw_step_time_ = t;
        yaw_step_size_ = delta;
    }
    // Relocate the target at time t (forces re-seek so a stale calibration
    // matters again after a station-keeping disturbance; review #1).
    void setTargetStep(double t, const Vec2& offset) {
        target_step_time_ = t;
        target_offset_ = offset;
    }
    double effectiveRelayYaw(double t) const {
        return (yaw_step_time_ >= 0.0 && t >= yaw_step_time_)
                   ? geo_.relay_yaw + yaw_step_size_
                   : geo_.relay_yaw;
    }
    double trueThetaAt(double t) const {
        return wrapAngle(effectiveRelayYaw(t) - geo_.heading0);
    }

  private:
    double noiseCh(double sigma, bool outlier) {
        double n = rng_.gauss(sigma);
        if (outlier)
            n += (rng_.uni(0.0, 1.0) < 0.5 ? -1.0 : 1.0) * re_.outlier_scale * sigma;
        return n;
    }

    struct Pending {
        double deliver_at;
        RelayPacket pkt;
    };

    TrialGeometry geo_;
    OdomErrorParams oe_;
    RelayErrorParams re_;
    double dt_;
    Rng rng_;
    Vec2 q_;
    double heading_;
    std::deque<Pending> queue_;
    double yaw_step_time_ = -1.0;
    double yaw_step_size_ = 0.0;
    double target_step_time_ = -1.0;
    Vec2 target_offset_{0, 0};
};

}  // namespace gfs

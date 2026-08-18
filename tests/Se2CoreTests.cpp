// Acceptance tests for the Campaign 2027 SE(2) core (plan §10).
// Dependency-free; exits nonzero on failure.

#include <cstdio>
#include <cstdlib>
#include <vector>

#include "gps_free_seeking/Campaign.hpp"
#include "gps_free_seeking/FixedLagSmoother.hpp"
#include "gps_free_seeking/Registration.hpp"
#include "gps_free_seeking/Supervisor.hpp"
#include "gps_free_seeking/WorldSE2.hpp"

using namespace gfs;

static int g_failures = 0;

#define CHECK(cond, msg)                                                \
    do {                                                                \
        if (!(cond)) {                                                  \
            std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
            ++g_failures;                                               \
        } else {                                                        \
            std::printf("ok  : %s\n", msg);                             \
        }                                                               \
    } while (0)

namespace {

RelayPacket makePacket(double t, double theta, const Vec2& s, const Vec2& x_o,
                       const Vec2& p_o, double sr = 1e-4, double sb = 1e-4) {
    RelayPacket p;
    p.t = t;
    const Vec2 lv = rotateT(theta, s - x_o);
    const Vec2 lt = rotateT(theta, p_o - x_o);
    p.r_v = lv.norm();
    p.beta_v = std::atan2(lv.y, lv.x);
    p.r_t = lt.norm();
    p.beta_t = std::atan2(lt.y, lt.x);
    p.sigma_r = sr;
    p.sigma_beta = sb;
    p.valid = true;
    return p;
}

struct TrialResult {
    double final_dist;
    double theta_err;
    Mode final_mode;
    int retriggers;
    std::vector<double> dist_trace;
};

// One closed-loop trial of the full pipeline against WorldSE2. The estimator
// sees only odometry samples and relay packets.
TrialResult runTrial(const TrialGeometry& geo, const OdomErrorParams& oe,
                     const RelayErrorParams& re, unsigned seed,
                     double T = 90.0) {
    const double dt = 0.01, meas_period = 0.05;
    WorldSE2 world(geo, oe, re, dt, seed);
    SupervisorConfig scfg;
    RegistrationConfig rcfg;
    GfsPipeline pipe(scfg, rcfg);

    TrialResult out{};
    SeekCommand cmd;  // zero until first packet
    const int steps = static_cast<int>(T / dt);
    const int meas_every = static_cast<int>(meas_period / dt);
    double tail = 0.0;
    int tail_n = 0;
    for (int k = 0; k <= steps; ++k) {
        const double t = k * dt;
        if (k % meas_every == 0) world.emitPacket(t);
        RelayPacket pkt;
        while (world.deliverDue(t, &pkt)) pipe.onPacket(pkt);
        cmd = pipe.command();
        const OdometrySample o = world.step(cmd, t);
        pipe.onOdometry(o);
        if (k % 100 == 0) out.dist_trace.push_back(world.distToTarget());
        if (t >= T - 10.0) {
            tail += world.distToTarget();
            ++tail_n;
        }
    }
    out.final_dist = tail / std::max(1, tail_n);
    out.theta_err = std::fabs(
        wrapAngle(pipe.estimate().theta_hat - world.trueTheta()));
    out.final_mode = pipe.mode();
    out.retriggers = pipe.retriggers();
    return out;
}

TrialGeometry fixedGeometry() {
    TrialGeometry g;
    g.q0 = Vec2{0, 0};
    g.heading0 = 0.4;
    g.target = Vec2{9.0, 6.0};
    g.relay = Vec2{5.0, -3.0};
    g.relay_yaw = 2.2;
    g.gamma = -1.1;
    return g;
}

// ---------------------------------------------------------------------------

void testTwoViewExactRecovery() {
    const double theta = 1.3;
    const Vec2 x_o{2.0, -1.0}, p_o{8.0, 5.0};
    WeightedWindowRegistration reg;
    reg.addView(0.0, Vec2{0, 0}, makePacket(0.0, theta, Vec2{0, 0}, x_o, p_o));
    reg.addView(0.5, Vec2{1.0, 0.7},
                makePacket(0.5, theta, Vec2{1.0, 0.7}, x_o, p_o));
    double th, var, info, spread, corr;
    CHECK(reg.solve(&th, &var, &info, &spread, &corr), "two-view solve succeeds");
    CHECK(std::fabs(wrapAngle(th - theta)) < 1e-9,
          "two-view theta exact (Theorem 2)");
    CHECK(corr > 0.999999, "noiseless correlation ~ 1");
}

void testRepeatedPoseRejected() {
    const double theta = -0.7;
    const Vec2 x_o{3.0, 2.0}, p_o{-4.0, 6.0};
    WeightedWindowRegistration reg;
    for (int i = 0; i < 5; ++i)
        reg.addView(0.1 * i, Vec2{1.0, 1.0},
                    makePacket(0.1 * i, theta, Vec2{1.0, 1.0}, x_o, p_o));
    double th, var, info, spread, corr;
    CHECK(!reg.solve(&th, &var, &info, &spread, &corr),
          "repeated pose rejected as unobservable (Corollary repeated-pose)");
}

void testDeliveredVariancePredictsError() {
    // Empirical yaw variance should track the first-order variance delivered
    // by the implemented weighted registration within a factor-of-2 band.
    const double theta = 0.9;
    const Vec2 x_o{1.0, 0.5}, p_o{6.0, 4.0};
    const double sr = 0.05, sb = 0.01;
    Rng rng(5);
    std::vector<Vec2> poses = {{0, 0}, {1.5, 0.3}, {2.5, 1.8}, {1.0, 3.0},
                               {3.5, 3.2}, {4.0, 1.0}};
    // P0 fix: average the predicted variance over realizations instead of
    // comparing against only the last realization's prediction.
    double pred_var_sum = 0.0;
    int pred_n = 0;
    std::vector<double> errs;
    for (int mc = 0; mc < 400; ++mc) {
        WeightedWindowRegistration reg;
        for (size_t i = 0; i < poses.size(); ++i) {
            RelayPacket p =
                makePacket(0.1 * i, theta, poses[i], x_o, p_o, sr, sb);
            p.r_v += rng.gauss(sr);
            p.beta_v += rng.gauss(sb);
            p.r_t += rng.gauss(sr);
            p.beta_t += rng.gauss(sb);
            reg.addView(0.1 * i, poses[i], p);
        }
        double th, var, info, spread, corr;
        if (!reg.solve(&th, &var, &info, &spread, &corr)) continue;
        errs.push_back(wrapAngle(th - theta));
        pred_var_sum += var;
        ++pred_n;
    }
    double var = 0.0;
    for (double e : errs) var += e * e;
    var /= errs.size();
    const double mean_pred = pred_var_sum / std::max(1, pred_n);
    const double ratio = var / mean_pred;  // ~1 when certificate is honest
    std::printf("      delivered-vs-empirical ratio = %.3f\n", ratio);
    CHECK(ratio > 0.5 && ratio < 2.0,
          "delivered yaw variance matches empirical variance within 2x");
}

void testCorrelatedOdometryVariancePredictsError() {
    const double theta = 1.1;
    const Vec2 x_o{2.0, 1.0}, p_o{9.0, 3.0};
    const double sr = 0.10, sb = 0.0175, sigma_step = 0.02;
    const int n_views = 16, steps_per_view = 5;
    std::vector<Vec2> truth;
    for (int i = 0; i < n_views; ++i) {
        const double u = static_cast<double>(i) / (n_views - 1);
        truth.push_back(Vec2{2.0 * u, 0.6 * std::sin(kPi * u)});
    }
    Rng rng(71234);
    double pred_sum = 0.0, mse = 0.0;
    int n = 0;
    for (int mc = 0; mc < 500; ++mc) {
        WeightedWindowRegistration reg;
        Vec2 s_hat = truth.front();
        double pose_var = 0.0;
        for (int i = 0; i < n_views; ++i) {
            if (i > 0) {
                const double inc_sigma = sigma_step * std::sqrt(steps_per_view);
                s_hat += truth[i] - truth[i - 1] +
                         Vec2{rng.gauss(inc_sigma), rng.gauss(inc_sigma)};
                pose_var += steps_per_view * sigma_step * sigma_step;
            }
            RelayPacket p = makePacket(0.05 * i, theta, truth[i], x_o, p_o,
                                       sr, sb);
            p.r_v += rng.gauss(sr);
            p.beta_v += rng.gauss(sb);
            p.r_t += rng.gauss(sr);
            p.beta_t += rng.gauss(sb);
            reg.addView(p.t, s_hat, p, pose_var);
        }
        double th, var, info, spread, corr;
        if (!reg.solve(&th, &var, &info, &spread, &corr)) continue;
        const double err = wrapAngle(th - theta);
        mse += err * err;
        pred_sum += var;
        ++n;
    }
    const double ratio = mse / std::max(pred_sum, 1e-18);
    std::printf("      odometry-correlation variance ratio = %.3f\n", ratio);
    CHECK(n == 500 && ratio > 0.75 && ratio < 1.25,
          "correlated odometry certificate matches empirical yaw error");
}

void advanceStraight(GfsPipeline* pipe, double t, double dx) {
    OdometrySample o;
    o.t = t;
    o.dxy = Vec2{dx, 0.0};
    o.dtheta = 0.0;
    o.sigma_xy = 0.0;
    o.sigma_dtheta = 0.0;
    pipe->onOdometry(o);
}

void testDelayedPacketCompensationAndMonotoneTiming() {
    SupervisorConfig cfg;
    cfg.theta_drift_rate = 1.0;
    RegistrationConfig no_registration;
    no_registration.min_packets = 100;
    GfsPipeline pipe(cfg, no_registration);
    const double theta = 0.6;
    const Vec2 x_o{2.0, -1.0}, p_o{8.0, 4.0};
    pipe.setOracleYaw(theta);

    // A packet measured at s=0 is delivered after the vehicle reaches s=1.
    const RelayPacket delayed =
        makePacket(0.0, theta, Vec2{0, 0}, x_o, p_o);
    advanceStraight(&pipe, 1.0, 1.0);
    pipe.onPacket(delayed);
    const Vec2 expected = p_o - Vec2{1.0, 0.0};
    CHECK((pipe.estimate().e_raw - expected).norm() < 1e-9,
          "delayed packet is compensated to the current vehicle pose");

    // Measurement timestamps now go backward, while delivery time advances.
    // The drift variance must accumulate two one-second delivery intervals,
    // not a negative interval followed by an inflated timestamp jump.
    advanceStraight(&pipe, 2.0, 0.0);
    pipe.onPacket(makePacket(-1.0, theta, Vec2{0, 0}, x_o, p_o));
    advanceStraight(&pipe, 3.0, 0.0);
    pipe.onPacket(makePacket(3.0, theta, Vec2{1, 0}, x_o, p_o));
    CHECK(std::fabs(pipe.controlVariance() - 2.0) < 1e-6,
          "out-of-order packets use monotone delivery-time variance growth");
}

void testChangePersistenceResetsOnUncertifiedWindow() {
    SupervisorConfig cfg;
    cfg.change_adopt_after = 2;
    cfg.change_min_duration = 0.0;
    cfg.change_min_interval = 0.0;
    cfg.min_window_packets = 2;
    cfg.cert_on_rad = 0.2;
    RegistrationConfig rcfg;
    rcfg.max_packets = 2;
    rcfg.min_packets = 2;
    GfsPipeline pipe(cfg, rcfg);
    pipe.setOracleYaw(0.0);

    const double changed_theta = 0.8;
    const Vec2 x_o{2.0, -1.0}, p_o{8.0, 4.0};
    pipe.onPacket(makePacket(0.0, changed_theta, Vec2{0, 0}, x_o, p_o));
    advanceStraight(&pipe, 1.0, 1.0);
    pipe.onPacket(makePacket(1.0, changed_theta, Vec2{1, 0}, x_o, p_o));
    CHECK(pipe.pendingChangeVotes() == 1,
          "first certified inconsistent window starts a change streak");

    // Repeated pose makes the two-entry window unobservable and must erase
    // the pending vote.
    advanceStraight(&pipe, 2.0, 0.0);
    pipe.onPacket(makePacket(2.0, changed_theta, Vec2{1, 0}, x_o, p_o));
    CHECK(pipe.pendingChangeVotes() == 0 && pipe.adoptions() == 0,
          "uncertified window breaks consecutive change evidence");

    advanceStraight(&pipe, 3.0, 1.0);
    pipe.onPacket(makePacket(3.0, changed_theta, Vec2{2, 0}, x_o, p_o));
    CHECK(pipe.pendingChangeVotes() == 1 && pipe.adoptions() == 0,
          "change persistence restarts after the evidence gap");
    advanceStraight(&pipe, 4.0, 1.0);
    pipe.onPacket(makePacket(4.0, changed_theta, Vec2{3, 0}, x_o, p_o));
    CHECK(pipe.adoptions() == 1,
          "coherent consecutive certified windows adopt the changed yaw");
}

void testStationTaskEventScoring() {
    OdomErrorParams oe;
    oe.bias_vel = Vec2{0.01, -0.005};
    RelayErrorParams re;
    const TrialMetrics m = runCampaignTrial(
        MethodKind::kProposed, fixedGeometry(), oe, re, 9191, 180.0, 60.0,
        40.0 * kPi / 180.0, 75.0, Vec2{2.0, -1.5});
    CHECK(m.recovery_class != "not_applicable",
          "yaw disturbance receives an explicit recovery classification");
    if (m.post_success)
        CHECK(m.post_time_to_goal >= 0.0,
              "post-success cannot pre-latch before target relocation");
}

void testSe2Invariance() {
    // Common SE(2) transform of the whole world (and gamma) must leave every
    // vehicle-side signal and the task error identical (Theorem 1).
    OdomErrorParams oe;  // defaults include noise: same seed -> same draws
    RelayErrorParams re;
    TrialGeometry a = fixedGeometry();
    TrialGeometry b = a;
    const double alpha = 0.85;
    const Vec2 c{40.0, -17.0};
    b.q0 = c + rotate(alpha, a.q0);
    b.target = c + rotate(alpha, a.target);
    b.relay = c + rotate(alpha, a.relay);
    b.relay_yaw = wrapAngle(a.relay_yaw + alpha);
    b.heading0 = wrapAngle(a.heading0 + alpha);
    b.gamma = wrapAngle(a.gamma + alpha);

    const TrialResult ra = runTrial(a, oe, re, 77, 60.0);
    const TrialResult rb = runTrial(b, oe, re, 77, 60.0);
    double max_diff = 0.0;
    for (size_t i = 0; i < ra.dist_trace.size(); ++i)
        max_diff = std::max(max_diff,
                            std::fabs(ra.dist_trace[i] - rb.dist_trace[i]));
    CHECK(max_diff < 1e-6, "SE(2) invariance of closed-loop task error");
    CHECK(std::fabs(ra.theta_err - rb.theta_err) < 1e-6,
          "SE(2) invariance of theta error");
}

void testSupervisorHysteresis() {
    SupervisorConfig cfg;
    cfg.min_dwell = 0.0;
    ExcitationSupervisor sup(cfg);
    EstimatorOutput est;
    est.valid = true;
    est.correlation = 1.0;
    est.window_packets = 20;
    bool rt;

    auto varFor = [](double halfwidth_deg) {
        const double hw = halfwidth_deg * kPi / 180.0;
        return (hw / 1.96) * (hw / 1.96);
    };

    // Certify over 3 windows -> SEEK.
    est.theta_variance = varFor(5.0);
    Mode m = Mode::kExcite;
    for (int i = 0; i < 3; ++i) m = sup.update(0.1 * i, est, 5.0, &rt);
    CHECK(m == Mode::kSeek, "supervisor certifies after 3 good windows");

    // 12 deg halfwidth: inside hysteresis band, stays SEEK.
    est.theta_variance = varFor(12.0);
    m = sup.update(0.4, est, 5.0, &rt);
    CHECK(m == Mode::kSeek && !rt, "hysteresis band holds SEEK (no chatter)");

    // 20 deg: retrigger to EXCITE.
    est.theta_variance = varFor(20.0);
    m = sup.update(0.5, est, 5.0, &rt);
    CHECK(m == Mode::kExcite && rt, "uncertainty growth retriggers EXCITE");

    // Re-certify, approach goal -> MAINTAIN, leave band -> SEEK.
    est.theta_variance = varFor(5.0);
    for (int i = 0; i < 3; ++i) m = sup.update(0.6 + 0.1 * i, est, 5.0, &rt);
    m = sup.update(1.0, est, 0.15, &rt);
    CHECK(m == Mode::kMaintain, "MAINTAIN inside 0.20 m");
    m = sup.update(1.1, est, 0.32, &rt);
    CHECK(m == Mode::kSeek, "resume SEEK outside 0.30 m");
}

void testCommandSaturation() {
    SupervisorConfig cfg;
    for (double ex : {1e6, -1e6, 0.0}) {
        const SeekCommand c =
            seekCommand(Mode::kSeek, Vec2{ex, ex}, 2.0, cfg, false);
        CHECK(std::fabs(c.v) <= cfg.v_max + 1e-12 &&
                  std::fabs(c.omega) <= cfg.omega_max + 1e-12,
              "final saturation bounds every command");
    }
}

void testSmootherNoiselessAndPsd() {
    const double theta = 2.9;  // near pi: exercises bearing wrap
    const Vec2 x_o{2.0, 1.0}, p_o{7.0, -2.0};
    FixedLagSmoother sm;
    Vec2 prev{0, 0};
    Rng rng(9);
    std::vector<Vec2> poses;
    for (int k = 0; k < 12; ++k)
        poses.push_back(Vec2{0.8 * k, 1.5 * std::sin(0.5 * k)});
    for (int k = 0; k < 12; ++k) {
        FixedLagSmoother::Node nd;
        nd.t = 0.5 * k;
        nd.s_init = poses[k] + Vec2{rng.gauss(0.02), rng.gauss(0.02)};
        nd.pkt = makePacket(nd.t, theta, poses[k], x_o, p_o, 0.02, 0.004);
        nd.odo_delta = (k == 0) ? Vec2{0, 0} : poses[k] - poses[k - 1];
        nd.odo_sigma = 0.02;
        nd.dt_prev = (k == 0) ? 0.0 : 0.5;
        sm.addNode(nd);
    }
    sm.setInitialGuess(theta + 0.3, x_o + Vec2{0.5, -0.4}, p_o + Vec2{-0.6, 0.5});
    CHECK(sm.solve(), "smoother GN converges");
    CHECK(std::fabs(wrapAngle(sm.theta() - theta)) < 5e-3,
          "smoother theta accurate (bearing-wrap regime)");
    CHECK((sm.pO() - p_o).norm() < 5e-2, "smoother target accurate");
    CHECK(sm.thetaVariance() > 0.0 && sm.thetaVariance() < 1.0,
          "smoother variance sane");
    CHECK(isPsd(sm.headInformation()), "carried-over prior information is PSD");
}

void testDeterministicReplay() {
    OdomErrorParams oe;
    RelayErrorParams re;
    re.dropout = 0.1;
    re.delay = 0.1;
    const TrialResult r1 = runTrial(fixedGeometry(), oe, re, 123, 60.0);
    const TrialResult r2 = runTrial(fixedGeometry(), oe, re, 123, 60.0);
    CHECK(r1.final_dist == r2.final_dist && r1.theta_err == r2.theta_err,
          "deterministic replay: identical outputs for identical seeds");
}

void testClosedLoopReach() {
    OdomErrorParams oe;
    oe.bias_vel = Vec2{0.01, -0.005};
    RelayErrorParams re;  // 0.10 m / 1 deg base
    int success = 0;
    Rng rng(2026);
    for (int i = 0; i < 10; ++i) {
        const TrialGeometry g = sampleGeometry(rng);
        const TrialResult r = runTrial(g, oe, re, 300 + i, 120.0);
        if (r.final_dist < 0.35) ++success;
        else std::printf("      trial %d final_dist=%.3f theta_err=%.3f\n",
                         i, r.final_dist, r.theta_err);
    }
    std::printf("      closed-loop reach %d/10\n", success);
    CHECK(success >= 9, "closed-loop reach over randomized geometry");
}

void testStressDegradations() {
    OdomErrorParams oe;
    oe.bias_vel = Vec2{0.01, 0.0};
    oe.heading_drift = 0.002;  // rad/s
    RelayErrorParams re;
    re.dropout = 0.3;
    re.delay = 0.2;
    re.delay_jitter = 0.1;
    re.outlier_prob = 0.05;
    const TrialResult r = runTrial(fixedGeometry(), oe, re, 555, 150.0);
    std::printf("      stress final_dist=%.3f retriggers=%d\n", r.final_dist,
                r.retriggers);
    CHECK(r.final_dist < 0.6, "reach under dropout+delay+outliers+drift");
}

}  // namespace

int main() {
    testTwoViewExactRecovery();
    testRepeatedPoseRejected();
    testDeliveredVariancePredictsError();
    testCorrelatedOdometryVariancePredictsError();
    testDelayedPacketCompensationAndMonotoneTiming();
    testChangePersistenceResetsOnUncertifiedWindow();
    testStationTaskEventScoring();
    testSe2Invariance();
    testSupervisorHysteresis();
    testCommandSaturation();
    testSmootherNoiselessAndPsd();
    testDeterministicReplay();
    testClosedLoopReach();
    testStressDegradations();
    if (g_failures == 0) {
        std::printf("all SE(2) core tests passed\n");
        return 0;
    }
    std::printf("%d test(s) failed\n", g_failures);
    return 1;
}

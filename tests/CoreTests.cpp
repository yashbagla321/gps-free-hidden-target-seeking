// Unit and regression tests for the GPS-free seeking project.
// Dependency-free: exits nonzero on first failure.

#include <cstdio>
#include <cstdlib>

#include "gps_free_seeking/Scenarios.hpp"

using namespace gfs;

static int g_failures = 0;

#define CHECK(cond, msg)                                            \
    do {                                                            \
        if (!(cond)) {                                              \
            std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
            ++g_failures;                                           \
        } else {                                                    \
            std::printf("ok  : %s\n", msg);                         \
        }                                                           \
    } while (0)

// Rotation identities used throughout the derivations.
static void testRotations() {
    const Vec2 v{1.3, -0.7};
    const double a = 0.9;
    const Vec2 w = rotate(a, rotateT(a, v));
    CHECK(std::fabs(w.x - v.x) < 1e-12 && std::fabs(w.y - v.y) < 1e-12,
          "R(a) R(a)^T = I");

    // d/da [R(a)^T v] equals R(a)^T S^T v with S = [[0,-1],[1,0]].
    const double h = 1e-7;
    const Vec2 num = (rotateT(a + h, v) - rotateT(a - h, v)) * (0.5 / h);
    const Vec2 ana = rotateT(a, Vec2{v.y, -v.x});
    CHECK((num - ana).norm() < 1e-6, "rotation Jacobian identity");

    CHECK(std::fabs(wrapAngle(3.0 * kPi)) - kPi < 1e-9, "angle wrap");
}

// In the noiseless case the yaw estimate and the relative target estimate
// must converge and the vehicle must reach the target.
static void testNoiselessConvergence() {
    SimParams prm;
    prm.noise = NoiseParams{};
    prm.noise.sigma_range = 0.0;
    prm.noise.sigma_bearing = 0.0;
    prm.noise.sigma_odo = 0.0;
    prm.noise.odo_bias = Vec2{0.0, 0.0};
    const RunSummary r = runRelativeSeeker(prm, 1u);
    CHECK(r.final_psi_err < 1e-2, "noiseless yaw convergence");
    CHECK(r.final_dist < 0.05, "noiseless target reaching");
    CHECK(r.reached, "noiseless reach flag");
}

// Translation gauge invariance: shifting the entire world by a constant must
// leave every vehicle-side signal identical (the estimator never sees
// absolute positions). We verify the closed-loop distance profile matches.
static void testGaugeInvariance() {
    SimParams a;
    a.noise.sigma_range = 0.0;
    a.noise.sigma_bearing = 0.0;
    a.noise.sigma_odo = 0.0;
    a.noise.odo_bias = Vec2{0.0, 0.0};
    SimParams b = a;
    const Vec2 shift{137.0, -58.0};
    b.q0 += shift;
    b.target += shift;
    b.beacon += shift;

    const RunSummary ra = runRelativeSeeker(a, 3u);
    const RunSummary rb = runRelativeSeeker(b, 3u);
    CHECK(std::fabs(ra.final_dist - rb.final_dist) < 1e-9,
          "translation gauge invariance of task error");
    CHECK(std::fabs(ra.final_psi_err - rb.final_psi_err) < 1e-9,
          "translation gauge invariance of yaw error");
}

// Moderate noise must still yield sub-meter station keeping.
static void testNoisyReach() {
    SimParams prm;  // default noise
    const RunSummary r = runRelativeSeeker(prm, 11u);
    CHECK(r.final_dist < 0.6, "noisy final distance < 0.6 m");
    CHECK(r.reached, "noisy reach flag");
}

// The EKF baseline with good GPS should also solve the task; this guards
// against strawman comparisons.
static void testEkfWithGoodGps() {
    SimParams prm;
    prm.noise.sigma_gps = 0.5;
    double abs_err = 0.0;
    const RunSummary r = runEkfSeeker(prm, 21u, true, &abs_err);
    CHECK(r.final_dist < 1.0, "EKF with 0.5 m GPS reaches target");
    CHECK(abs_err < 1.5, "EKF absolute target error small with good GPS");
}

// Odometry bias must not accumulate into the task error: doubling the horizon
// should not grow the tail distance.
static void testDriftBounded() {
    SimParams prm;
    prm.noise.odo_bias = Vec2{0.02, -0.01};
    prm.T = 300.0;
    const RunSummary r1 = runRelativeSeeker(prm, 33u);
    prm.T = 600.0;
    const RunSummary r2 = runRelativeSeeker(prm, 33u);
    CHECK(r2.final_dist < r1.final_dist + 0.3,
          "task error does not accumulate with horizon under odometry bias");
}

int main() {
    testRotations();
    testNoiselessConvergence();
    testGaugeInvariance();
    testNoisyReach();
    testEkfWithGoodGps();
    testDriftBounded();
    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return 0;
    }
    std::printf("%d test(s) failed\n", g_failures);
    return 1;
}

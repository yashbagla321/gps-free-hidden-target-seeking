// Campaign 2027 smoke driver: one full-stack closed-loop trial with the
// registration estimator, EXCITE-SEEK-MAINTAIN supervisor, nonholonomic
// controller, and the fixed-lag smoother running through its acceptance gate.
//
// This is the Gate-B end-to-end wiring check, not the campaign runner (the
// campaign runner lands with plan §5 once configs are frozen).
//
// Usage: gps_free_seeking_campaign2027_smoke [out_csv] [seed]

#include <cstdio>
#include <fstream>
#include <string>

#include "gps_free_seeking/FixedLagSmoother.hpp"
#include "gps_free_seeking/Supervisor.hpp"
#include "gps_free_seeking/WorldSE2.hpp"

using namespace gfs;

int main(int argc, char** argv) {
    const std::string out =
        argc > 1 ? argv[1] : "results/campaign2027/offline/smoke_trial.csv";
    const unsigned seed = argc > 2 ? std::atoi(argv[2]) : 42u;

    Rng grng(seed);
    const TrialGeometry geo = sampleGeometry(grng);
    OdomErrorParams oe;
    oe.bias_vel = Vec2{0.01, -0.005};
    RelayErrorParams re;  // base: 0.10 m, 1 deg
    const double dt = 0.01, T = 150.0;
    WorldSE2 world(geo, oe, re, dt, seed);

    SupervisorConfig scfg;
    RegistrationConfig rcfg;
    GfsPipeline pipe(scfg, rcfg);
    FixedLagSmoother smoother;
    int packets_since_solve = 0;
    int smoother_accepts = 0, smoother_solves = 0;
    double last_pkt_t = 0.0;
    Vec2 last_pkt_s{0, 0};
    bool have_prev_pkt = false;

    std::ofstream f(out);
    f << "t,mode,dist_true,e_hat_norm,theta_err_ctrl,cert_halfwidth_deg,"
         "spread,correlation,smoother_active,retriggers\n";

    for (int k = 0; k <= static_cast<int>(T / dt); ++k) {
        const double t = k * dt;
        if (k % 5 == 0) world.emitPacket(t);  // 20 Hz
        RelayPacket pkt;
        while (world.deliverDue(t, &pkt)) {
            pipe.onPacket(pkt);
            // Feed the smoother the same packet stream + integrated pose.
            FixedLagSmoother::Node nd;
            nd.t = pkt.t;
            nd.s_init = pipe.s();
            nd.pkt = pkt;
            nd.odo_delta = have_prev_pkt ? pipe.s() - last_pkt_s : Vec2{0, 0};
            nd.odo_sigma = 0.01;
            nd.dt_prev = have_prev_pkt ? pkt.t - last_pkt_t : 0.0;
            smoother.addNode(nd);
            last_pkt_t = pkt.t;
            last_pkt_s = pipe.s();
            have_prev_pkt = true;

            if (++packets_since_solve >= 5 && pipe.estimate().valid) {
                packets_since_solve = 0;
                const auto& est = pipe.estimate();
                const Vec2 x0 = pipe.s() - rotate(est.theta_hat, pkt.lv());
                const Vec2 p0 = x0 + rotate(est.theta_hat, pkt.lt());
                smoother.setInitialGuess(est.theta_hat, x0, p0);
                ++smoother_solves;
                if (smoother.solve()) {
                    pipe.acceptSmoother(smoother.theta(),
                                        smoother.thetaVariance());
                    ++smoother_accepts;
                }
            }
        }
        const OdometrySample o = world.step(pipe.command(), t);
        pipe.onOdometry(o);

        if (k % 50 == 0) {
            const auto& e = pipe.estimate();
            f << t << ',' << modeName(pipe.mode()) << ','
              << world.distToTarget() << ',' << e.e_hat.norm() << ','
              << std::fabs(wrapAngle(e.theta_hat - world.trueTheta())) << ','
              << 1.96 * std::sqrt(e.theta_variance) * 180.0 / kPi << ','
              << e.spread << ',' << e.correlation << ','
              << (e.smoother_active ? 1 : 0) << ',' << pipe.retriggers()
              << '\n';
        }
    }
    std::printf(
        "smoke: final_dist=%.3f m, mode=%s, retriggers=%d, "
        "smoother %d/%d solves accepted, csv=%s\n",
        world.distToTarget(), modeName(pipe.mode()), pipe.retriggers(),
        smoother_accepts, smoother_solves, out.c_str());
    return 0;
}

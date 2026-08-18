// Dumps representative closed-loop traces for the submission video.
// Visualization only: statistics in the paper come exclusively from the
// provenance-locked campaign, never from these traces.
//
// Usage: viz_dump <out.csv> <scenario: nominal|yawstep> [seed]

#include <cstdio>
#include <cstring>
#include <fstream>

#include "gps_free_seeking/Supervisor.hpp"
#include "gps_free_seeking/WorldSE2.hpp"

using namespace gfs;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: viz_dump <out.csv> <nominal|yawstep> [seed]\n");
        return 1;
    }
    const std::string out = argv[1];
    const bool yawstep = std::strcmp(argv[2], "yawstep") == 0;
    const unsigned seed = argc > 3 ? std::atoi(argv[3]) : 12u;

    Rng grng(seed);
    TrialGeometry geo = sampleGeometry(grng);
    OdomErrorParams oe;
    oe.bias_vel = Vec2{0.01, -0.005};
    RelayErrorParams re;
    const double dt = 0.01, T = yawstep ? 90.0 : 70.0;
    WorldSE2 world(geo, oe, re, dt, seed);
    if (yawstep) world.setYawStep(8.0, 60.0 * kPi / 180.0);

    SupervisorConfig scfg;
    RegistrationConfig rcfg;
    GfsPipeline pipe(scfg, rcfg);

    Vec2 s_dr{0, 0};
    std::ofstream f(out);
    f << "t,qx,qy,drx,dry,px,py,rx,ry,relay_yaw,mode,dist,cert_hw_deg,"
         "theta_err_deg\n";
    const int steps = static_cast<int>(T / dt);
    for (int k = 0; k <= steps; ++k) {
        const double t = k * dt;
        if (k % 5 == 0) world.emitPacket(t);
        RelayPacket pkt;
        while (world.deliverDue(t, &pkt)) pipe.onPacket(pkt);
        const OdometrySample o = world.step(pipe.command(), t);
        s_dr += rotate(pipe.headingO(), o.dxy);
        pipe.onOdometry(o);
        if (k % 10 == 0) {  // 10 Hz trace
            const auto& est = pipe.estimate();
            // World-frame render: convert evaluator O-frame dead-reck pose
            // back to W for a common picture.
            const Vec2 dr_w =
                geo.q0 + rotate(geo.heading0, s_dr);
            f << t << ',' << world.q().x << ',' << world.q().y << ','
              << dr_w.x << ',' << dr_w.y << ',' << geo.target.x << ','
              << geo.target.y << ',' << geo.relay.x << ',' << geo.relay.y
              << ',' << world.effectiveRelayYaw(t) << ','
              << static_cast<int>(pipe.mode()) << ',' << world.distToTarget()
              << ',' << 1.96 * std::sqrt(std::max(est.theta_variance, 0.0)) *
                            180.0 / kPi
              << ','
              << std::fabs(wrapAngle(pipe.thetaCtrl() - world.trueThetaAt(t))) *
                     180.0 / kPi
              << '\n';
        }
    }
    std::printf("wrote %s (final dist %.3f m)\n", out.c_str(),
                world.distToTarget());
    return 0;
}

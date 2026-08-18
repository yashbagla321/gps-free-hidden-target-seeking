// GPS-free hidden-target seeking: simulation study driver.
//
// Scenarios
//   S1 nominal      : no GPS, moderate noise, closed-loop time series.
//   S2 noise MC     : Monte Carlo sweep over beacon noise scale.
//   S3 GPS sweep    : task error vs GPS noise for (a) proposed no-GPS method,
//                     (b) absolute EKF fusing GPS, (c) naive GPS-as-truth.
//   S4 excitation   : ablation with random initial yaw error, with/without
//                     exploratory excitation.
//   S5 drift        : odometry bias sweep and long-horizon station keeping;
//                     dead-reckoning error grows, task error stays bounded.
//
// Usage: gps_free_seeking_sim [results_dir]

#include <fstream>
#include <iostream>
#include <string>

#include "gps_free_seeking/Scenarios.hpp"

using namespace gfs;

namespace {

std::string g_dir = "results";

std::ofstream openCsv(const std::string& name) {
    std::ofstream f(g_dir + "/" + name);
    if (!f) {
        std::cerr << "cannot open " << g_dir + "/" + name << "\n";
        std::exit(1);
    }
    return f;
}

void scenarioNominal() {
    SimParams prm;
    auto f = openCsv("s1_nominal_timeseries.csv");
    RunOptions opt;
    opt.trace = &f;
    const RunSummary s = runRelativeSeeker(prm, 7u, opt);
    std::cout << "[S1] nominal: final_dist=" << s.final_dist
              << " m, psi_err=" << s.final_psi_err
              << " rad, time_to_reach=" << s.time_to_reach << " s\n";
}

void scenarioNoiseMc() {
    const double scales[] = {0.25, 0.5, 1.0, 2.0, 4.0};
    const int n_seeds = 40;
    auto f = openCsv("s2_noise_mc.csv");
    f << "noise_scale,sigma_range,sigma_bearing_deg,median_final_dist,"
         "q90_final_dist,median_psi_err,reach_rate\n";
    for (double s : scales) {
        SimParams prm;
        prm.noise.sigma_range = 0.10 * s;
        prm.noise.sigma_bearing = 0.0175 * s;
        std::vector<double> dists, psis;
        int reached = 0;
        for (int i = 0; i < n_seeds; ++i) {
            const RunSummary r = runRelativeSeeker(prm, 100u + i);
            dists.push_back(r.final_dist);
            psis.push_back(r.final_psi_err);
            reached += r.reached ? 1 : 0;
        }
        f << s << ',' << prm.noise.sigma_range << ','
          << prm.noise.sigma_bearing * 180.0 / kPi << ',' << median(dists) << ','
          << quantile(dists, 0.9) << ',' << median(psis) << ','
          << static_cast<double>(reached) / n_seeds << '\n';
        std::cout << "[S2] scale=" << s << " median_dist=" << median(dists)
                  << " reach=" << reached << "/" << n_seeds << "\n";
    }
}

void scenarioGpsSweep() {
    const double sigmas[] = {0.5, 1.0, 2.0, 5.0, 10.0, 20.0};
    const int n_seeds = 30;
    auto f = openCsv("s3_gps_sweep.csv");
    f << "sigma_gps,method,median_final_dist,q90_final_dist,reach_rate,"
         "median_abs_target_err\n";

    // Proposed method ignores GPS entirely: one row (constant vs sigma_gps).
    {
        SimParams prm;
        std::vector<double> dists;
        int reached = 0;
        for (int i = 0; i < n_seeds; ++i) {
            const RunSummary r = runRelativeSeeker(prm, 200u + i);
            dists.push_back(r.final_dist);
            reached += r.reached ? 1 : 0;
        }
        for (double sg : sigmas) {
            f << sg << ",proposed_no_gps," << median(dists) << ','
              << quantile(dists, 0.9) << ','
              << static_cast<double>(reached) / n_seeds << ",nan\n";
        }
        std::cout << "[S3] proposed: median_dist=" << median(dists) << "\n";
    }

    for (double sg : sigmas) {
        SimParams prm;
        prm.noise.sigma_gps = sg;
        std::vector<double> dists, abs_errs;
        int reached = 0;
        for (int i = 0; i < n_seeds; ++i) {
            double abs_err = 0.0;
            const RunSummary r = runEkfSeeker(prm, 200u + i, true, &abs_err);
            dists.push_back(r.final_dist);
            abs_errs.push_back(abs_err);
            reached += r.reached ? 1 : 0;
        }
        f << sg << ",ekf_gps," << median(dists) << ',' << quantile(dists, 0.9)
          << ',' << static_cast<double>(reached) / n_seeds << ','
          << median(abs_errs) << '\n';
        std::cout << "[S3] ekf sigma=" << sg << " median_dist=" << median(dists)
                  << " abs_target_err=" << median(abs_errs) << "\n";
    }

    for (double sg : sigmas) {
        SimParams prm;
        prm.noise.sigma_gps = sg;
        std::vector<double> dists;
        int reached = 0;
        for (int i = 0; i < n_seeds; ++i) {
            const RunSummary r = runNaiveGpsSeeker(prm, 200u + i);
            dists.push_back(r.final_dist);
            reached += r.reached ? 1 : 0;
        }
        f << sg << ",naive_gps_as_truth," << median(dists) << ','
          << quantile(dists, 0.9) << ','
          << static_cast<double>(reached) / n_seeds << ",nan\n";
        std::cout << "[S3] naive sigma=" << sg << " median_dist=" << median(dists)
                  << "\n";
    }
}

void scenarioExcitationAblation() {
    const int n_seeds = 100;
    auto f = openCsv("s4_excitation_ablation.csv");
    f << "excitation,seed,psi_hat0_err,final_dist,time_to_reach,reached\n";
    for (int exc = 0; exc < 2; ++exc) {
        int reached = 0;
        for (int i = 0; i < n_seeds; ++i) {
            SimParams prm;
            Rng rng(900u + i);
            RunOptions opt;
            opt.excitation = (exc == 1);
            opt.psi_hat0 = wrapAngle(prm.beacon_yaw + rng.uni(-kPi, kPi));
            const RunSummary r = runRelativeSeeker(prm, 900u + i, opt);
            f << exc << ',' << i << ','
              << std::fabs(wrapAngle(opt.psi_hat0 - prm.beacon_yaw)) << ','
              << r.final_dist << ',' << r.time_to_reach << ','
              << (r.reached ? 1 : 0) << '\n';
            reached += r.reached ? 1 : 0;
        }
        std::cout << "[S4] excitation=" << exc << " reach=" << reached << "/"
                  << n_seeds << "\n";
    }
}

void scenarioDrift() {
    // Long-horizon station keeping under odometry bias.
    const double biases[] = {0.0, 0.005, 0.01, 0.02, 0.05};
    auto f = openCsv("s5_drift_bias_sweep.csv");
    f << "odo_bias,median_final_dist,q90_final_dist,median_psi_err\n";
    const int n_seeds = 20;
    for (double b : biases) {
        SimParams prm;
        prm.T = 600.0;
        prm.noise.odo_bias = Vec2{b, -0.5 * b};
        std::vector<double> dists, psis;
        for (int i = 0; i < n_seeds; ++i) {
            const RunSummary r = runRelativeSeeker(prm, 500u + i);
            dists.push_back(r.final_dist);
            psis.push_back(r.final_psi_err);
        }
        f << b << ',' << median(dists) << ',' << quantile(dists, 0.9) << ','
          << median(psis) << '\n';
        std::cout << "[S5] bias=" << b << " median_dist=" << median(dists) << "\n";
    }

    // Time series with strong bias: task error bounded, dead reckoning drifts.
    SimParams prm;
    prm.T = 600.0;
    prm.noise.odo_bias = Vec2{0.02, -0.01};
    auto g = openCsv("s5_drift_timeseries.csv");
    RunOptions opt;
    opt.trace = &g;
    opt.trace_period = 1.0;
    const RunSummary s = runRelativeSeeker(prm, 42u, opt);
    std::cout << "[S5] long-horizon final_dist=" << s.final_dist << " m\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1) g_dir = argv[1];
    scenarioNominal();
    scenarioNoiseMc();
    scenarioGpsSweep();
    scenarioExcitationAblation();
    scenarioDrift();
    std::cout << "done. CSVs in " << g_dir << "\n";
    return 0;
}

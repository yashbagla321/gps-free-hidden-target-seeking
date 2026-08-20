// Campaign 2027 offline campaign runner (plan §5).
//
// Usage:
//   gps_free_seeking_campaign baselines [n_seeds] [method] [git_head]
//   gps_free_seeking_campaign conditioning [n_mc] [-] [git_head]
//   gps_free_seeking_campaign covariance_ablation [n_mc] [-] [git_head]
//
// Writes per-trial CSVs, per-method summaries, and provenance manifests into
// results/campaign2027/offline/.

#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

#include "gps_free_seeking/Campaign.hpp"

using namespace gfs;

namespace {

std::string g_out = "results/campaign2027/offline";  // per-run dir set in main
#define kOut g_out.c_str()

void studyBaselines(int n_seeds, const std::string& only_method,
                    const std::string& git_head) {
    const MethodKind kinds[] = {
        MethodKind::kProposed,           MethodKind::kProposedSmoother,
        MethodKind::kOracleYaw,          MethodKind::kEkf,
        MethodKind::kEndpointOnly,       MethodKind::kFixedDecayExc,
        MethodKind::kNoExcitation,       MethodKind::kProposedDiagVar,
        MethodKind::kProposedPacketOnly, MethodKind::kAnchoredNaive,
    };
    const unsigned seed_base = 10000;
    OdomErrorParams oe;
    oe.bias_vel = Vec2{0.01, -0.005};
    RelayErrorParams re;  // base sensing: 0.10 m, 1 deg

    std::ofstream trials(std::string(kOut) + "/s3_baselines_trials_" +
                         (only_method.empty() ? "all" : only_method) + ".csv");
    trials << "method,seed,success,failure,time_to_goal,station_rmse,"
              "theta_err,mean_runtime_us,deadreck_err,retriggers\n";
    std::ofstream summary(std::string(kOut) + "/s3_baselines_summary_" +
                          (only_method.empty() ? "all" : only_method) + ".csv");
    summary << "method,n,success_rate,wilson_lo,wilson_hi,median_ttg,"
               "median_station_rmse,rmse_ci_lo,rmse_ci_hi,q90_station_rmse,"
               "median_theta_err,median_runtime_us,never_certified,no_reach,"
               "hold_broken\n";

    for (MethodKind kind : kinds) {
        if (!only_method.empty() && only_method != methodName(kind)) continue;
        int succ = 0, f_cert = 0, f_reach = 0, f_hold = 0;
        std::vector<double> ttg, rmse, terr, rt;
        for (int i = 0; i < n_seeds; ++i) {
            Rng grng(seed_base + i);
            const TrialGeometry geo = sampleGeometry(grng);
            const TrialMetrics m =
                runCampaignTrial(kind, geo, oe, re, seed_base + i);
            trials << methodName(kind) << ',' << seed_base + i << ','
                   << (m.success ? 1 : 0) << ',' << m.failure << ','
                   << m.time_to_goal << ',' << m.station_rmse << ','
                   << m.theta_err << ',' << m.mean_runtime_us << ','
                   << m.deadreck_err << ',' << m.retriggers << '\n';
            succ += m.success ? 1 : 0;
            if (m.success && m.time_to_goal >= 0) ttg.push_back(m.time_to_goal);
            rmse.push_back(m.station_rmse);
            terr.push_back(m.theta_err);
            rt.push_back(m.mean_runtime_us);
            if (m.failure == "never_certified") ++f_cert;
            if (m.failure == "no_reach") ++f_reach;
            if (m.failure == "hold_broken") ++f_hold;
        }
        const WilsonInterval wi = wilson95(succ, n_seeds);
        double ci_lo, ci_hi;
        bootstrapMedianCi(rmse, &ci_lo, &ci_hi);
        summary << methodName(kind) << ',' << n_seeds << ','
                << static_cast<double>(succ) / n_seeds << ',' << wi.lo << ','
                << wi.hi << ',' << medianOf(ttg) << ',' << medianOf(rmse)
                << ',' << ci_lo << ',' << ci_hi << ','
                << quantileOf(rmse, 0.9) << ',' << medianOf(terr) << ','
                << medianOf(rt) << ',' << f_cert << ',' << f_reach << ','
                << f_hold << '\n';
        std::printf(
            "[S3] %-18s success=%3d/%d  med_rmse=%.3f  med_ttg=%.1f  "
            "med_rt=%.1fus  fail(cert/reach/hold)=%d/%d/%d\n",
            methodName(kind), succ, n_seeds, medianOf(rmse), medianOf(ttg),
            medianOf(rt), f_cert, f_reach, f_hold);
    }
    writeManifest(std::string(kOut) + "/s3_baselines_manifest_" +
                      (only_method.empty() ? "all" : only_method) + ".json",
                  "s3_baselines", git_head, n_seeds, seed_base, 150.0,
                  "\"relay_noise\": {\"sigma_r_m\": 0.10, "
                  "\"sigma_beta_rad\": 0.0175},\n  \"odometry\": {"
                  "\"sigma_xy_step_m\": 0.005, "
                  "\"sigma_dtheta_step_rad\": 0.001, "
                  "\"bias_velocity_m_s\": [0.01, -0.005]}");
}

// Theorem-linked study: I_theta certificate vs empirical yaw variance across
// window geometries (Lemma 1), plus the one-pose/two-view rank transition.
void studyConditioning(int n_mc, const std::string& git_head) {
    std::ofstream f(std::string(kOut) + "/s2_conditioning.csv");
    f << "chord_m,n_views,pred_var,emp_var,pred_info_bound,ratio,"
         "cov90,cov95,cov99,n_realizations\n";
    std::ofstream ftr(std::string(kOut) + "/s2_conditioning_trials.csv");
    ftr << "chord_m,n_views,mc,theta_err,pred_var,z\n";
    const double theta = 1.1;
    const Vec2 x_o{2.0, 1.0}, p_o{9.0, 3.0};
    const double sr = 0.10, sb = 0.0175;

    for (double chord : {0.25, 0.5, 1.0, 2.0, 4.0}) {
        for (int n_views : {2, 4, 8, 16}) {
            // Views along an arc of given chord length.
            std::vector<Vec2> poses;
            for (int i = 0; i < n_views; ++i) {
                const double u = static_cast<double>(i) / (n_views - 1);
                poses.push_back(Vec2{chord * u, 0.3 * chord * std::sin(kPi * u)});
            }
            Rng rng(99);
            std::vector<double> errs, zs;
            // P0 fix (2026-08-16 review): predictions are AVERAGED over the
            // Monte Carlo realizations; the previous code compared the
            // empirical variance against only the LAST realization's
            // prediction, which is statistically invalid.
            double pred_var_sum = 0.0, pred_info_sum = 0.0;
            int pred_n = 0;
            for (int mc = 0; mc < n_mc; ++mc) {
                WeightedWindowRegistration reg;
                for (size_t i = 0; i < poses.size(); ++i) {
                    RelayPacket p;
                    p.t = 0.1 * i;
                    const Vec2 lv = rotateT(theta, poses[i] - x_o);
                    const Vec2 lt = rotateT(theta, p_o - x_o);
                    p.r_v = lv.norm() + rng.gauss(sr);
                    p.beta_v = std::atan2(lv.y, lv.x) + rng.gauss(sb);
                    p.r_t = lt.norm() + rng.gauss(sr);
                    p.beta_t = std::atan2(lt.y, lt.x) + rng.gauss(sb);
                    p.sigma_r = sr;
                    p.sigma_beta = sb;
                    p.valid = true;
                    reg.addView(p.t, poses[i], p);
                }
                double th, var, info, spread, corr;
                if (!reg.solve(&th, &var, &info, &spread, &corr)) continue;
                const double err = wrapAngle(th - theta);
                errs.push_back(err);
                // Coverage of the per-realization first-order interval
                // (review #2: this is what the supervisor actually uses).
                const double z = err / std::sqrt(std::max(var, 1e-18));
                zs.push_back(z);
                ftr << chord << ',' << n_views << ',' << mc << ',' << err
                    << ',' << var << ',' << z << '\n';
                pred_var_sum += var;
                pred_info_sum += info;
                ++pred_n;
            }
            double emp = 0.0;
            for (double e : errs) emp += e * e;
            emp /= std::max<size_t>(1, errs.size());
            const double pred_var =
                pred_n ? pred_var_sum / pred_n : 0.0;
            const double pred_info =
                pred_n ? pred_info_sum / pred_n : 0.0;
            int c90 = 0, c95 = 0, c99 = 0;
            for (double z : zs) {
                if (std::fabs(z) <= 1.645) ++c90;
                if (std::fabs(z) <= 1.960) ++c95;
                if (std::fabs(z) <= 2.576) ++c99;
            }
            const auto nz = std::max<size_t>(1, zs.size());
            f << chord << ',' << n_views << ',' << pred_var << ',' << emp
              << ',' << 1.0 / std::max(pred_info, 1e-12) << ','
              << (pred_var > 0 ? emp / pred_var : 0.0) << ','
              << static_cast<double>(c90) / nz << ','
              << static_cast<double>(c95) / nz << ','
              << static_cast<double>(c99) / nz << ',' << zs.size() << '\n';
            std::printf(
                "[S2] chord=%.2f n=%2d ratio=%.2f cov95=%.3f cov99=%.3f\n",
                chord, n_views, pred_var > 0 ? emp / pred_var : 0.0,
                static_cast<double>(c95) / nz,
                static_cast<double>(c99) / nz);
        }
    }
    writeManifest(std::string(kOut) + "/s2_conditioning_manifest.json",
                  "s2_conditioning", git_head, n_mc, 99, 0.0,
                  "\"measurement_noise\": {\"sigma_r_m\": 0.10, "
                  "\"sigma_beta_rad\": 0.0175},\n  \"grid\": {"
                  "\"chord_m\": [0.25, 0.5, 1.0, 2.0, 4.0], "
                  "\"n_views\": [2, 4, 8, 16]}");
}

// Translation-random-walk calibration of the delivered yaw certificate.
// Unlike S2, the poses supplied to registration are noisy integrated
// odometry and carry their accumulated per-axis variance.
void studyOdometryCoverage(int n_mc, const std::string& git_head) {
    std::ofstream f(std::string(kOut) + "/s2_odometry_coverage.csv");
    f << "sigma_xy_step_m,n_views,pred_var,emp_mse,ratio,mean_error,"
         "cov90,cov95,cov99,n_realizations\n";
    std::ofstream ftr(std::string(kOut) +
                      "/s2_odometry_coverage_trials.csv");
    ftr << "sigma_xy_step_m,n_views,mc,theta_err,pred_var,z\n";
    const double theta = 1.1;
    const Vec2 x_o{2.0, 1.0}, p_o{9.0, 3.0};
    const double sr = 0.10, sb = 0.0175;
    const int steps_per_view = 5;
    int cell = 0;
    for (double sigma_step : {0.0025, 0.005, 0.01, 0.02, 0.05}) {
        for (int n_views : {8, 16}) {
            std::vector<Vec2> truth;
            for (int i = 0; i < n_views; ++i) {
                const double u = static_cast<double>(i) / (n_views - 1);
                truth.push_back(
                    Vec2{2.0 * u, 0.6 * std::sin(kPi * u)});
            }
            Rng rng(71000u + static_cast<unsigned>(cell++));
            std::vector<double> errs, zs;
            double pred_sum = 0.0;
            int pred_n = 0;
            for (int mc = 0; mc < n_mc; ++mc) {
                WeightedWindowRegistration reg;
                Vec2 s_hat = truth.front();
                double pose_var = 0.0;
                for (int i = 0; i < n_views; ++i) {
                    if (i > 0) {
                        const Vec2 ds = truth[i] - truth[i - 1];
                        const double inc_sigma =
                            sigma_step * std::sqrt(steps_per_view);
                        s_hat += ds +
                                 Vec2{rng.gauss(inc_sigma),
                                      rng.gauss(inc_sigma)};
                        pose_var +=
                            steps_per_view * sigma_step * sigma_step;
                    }
                    RelayPacket p;
                    p.t = 0.05 * i;
                    const Vec2 lv = rotateT(theta, truth[i] - x_o);
                    const Vec2 lt = rotateT(theta, p_o - x_o);
                    p.r_v = lv.norm() + rng.gauss(sr);
                    p.beta_v =
                        std::atan2(lv.y, lv.x) + rng.gauss(sb);
                    p.r_t = lt.norm() + rng.gauss(sr);
                    p.beta_t =
                        std::atan2(lt.y, lt.x) + rng.gauss(sb);
                    p.sigma_r = sr;
                    p.sigma_beta = sb;
                    p.valid = true;
                    reg.addView(p.t, s_hat, p, pose_var);
                }
                double th, var, info, spread, corr;
                if (!reg.solve(&th, &var, &info, &spread, &corr)) continue;
                const double err = wrapAngle(th - theta);
                const double z = err / std::sqrt(std::max(var, 1e-18));
                errs.push_back(err);
                zs.push_back(z);
                pred_sum += var;
                ++pred_n;
                ftr << sigma_step << ',' << n_views << ',' << mc << ','
                    << err << ',' << var << ',' << z << '\n';
            }
            double mse = 0.0, mean = 0.0;
            int c90 = 0, c95 = 0, c99 = 0;
            for (size_t i = 0; i < errs.size(); ++i) {
                mean += errs[i];
                mse += errs[i] * errs[i];
                if (std::fabs(zs[i]) <= 1.645) ++c90;
                if (std::fabs(zs[i]) <= 1.960) ++c95;
                if (std::fabs(zs[i]) <= 2.576) ++c99;
            }
            const auto nz = std::max<size_t>(1, errs.size());
            mean /= nz;
            mse /= nz;
            const double pred = pred_n ? pred_sum / pred_n : 0.0;
            f << sigma_step << ',' << n_views << ',' << pred << ',' << mse
              << ',' << (pred > 0.0 ? mse / pred : 0.0) << ',' << mean
              << ',' << static_cast<double>(c90) / nz << ','
              << static_cast<double>(c95) / nz << ','
              << static_cast<double>(c99) / nz << ',' << errs.size() << '\n';
            std::printf(
                "[S2-ODO] sigma=%.4f n=%d ratio=%.2f cov95=%.3f\n",
                sigma_step, n_views, pred > 0.0 ? mse / pred : 0.0,
                static_cast<double>(c95) / nz);
        }
    }
    writeManifest(
        std::string(kOut) + "/s2_odometry_coverage_manifest.json",
        "s2_odometry_coverage", git_head, n_mc, 71000, 0.0,
        "\"measurement_noise\": {\"sigma_r_m\": 0.10, "
        "\"sigma_beta_rad\": 0.0175},\n  \"odometry_grid\": {"
        "\"sigma_xy_step_m\": [0.0025, 0.005, 0.01, 0.02, 0.05], "
        "\"n_views\": [8, 16], \"steps_per_view\": 5}");
}

// Covariance-ablation study (review response, Phase 3): isolates the
// contribution of the paper's central claim -- modeling cross-view
// odometry correlation in closed form -- by running the identical
// estimator and controller under three certificate variance models
// (Registration.hpp VarianceModel): kFull (Theorem 3, the paper's
// certificate), kDiag (naive independent-pose covariance, dropping the
// shared-increment correlation), and kPacketOnly (pose term dropped
// entirely, as if odometry were exact). Reuses the S2 odometry-coverage
// grid for the offline variance-ratio/coverage/false-certification
// numbers, then adds one closed-loop success cell per model at the
// nominal condition. Expected story: kFull stays near nominal coverage;
// kDiag and kPacketOnly undercover.
void studyCovarianceAblation(int n_mc, const std::string& git_head) {
    struct ModelSpec {
        VarianceModel model;
        const char* name;
    };
    const ModelSpec models[] = {
        {VarianceModel::kFull, "full"},
        {VarianceModel::kDiag, "diag"},
        {VarianceModel::kPacketOnly, "packet_only"},
    };
    const double cert_on_rad = SupervisorConfig{}.cert_on_rad;

    std::ofstream f(std::string(kOut) + "/s9_covariance_ablation.csv");
    f << "model,sigma_xy_step_m,n_views,pred_var,emp_mse,ratio,cov90,cov95,"
         "cov99,false_cert_rate,n_realizations\n";
    std::ofstream ftr(std::string(kOut) +
                      "/s9_covariance_ablation_trials.csv");
    ftr << "model,sigma_xy_step_m,n_views,mc,theta_err,pred_var,z,"
           "would_certify,false_cert\n";

    const double theta = 1.1;
    const Vec2 x_o{2.0, 1.0}, p_o{9.0, 3.0};
    const double sr = 0.10, sb = 0.0175;
    const int steps_per_view = 5;

    // Pooled accumulators per model, across the whole grid (matching the
    // paper's "pooled" convention for Remark 1).
    struct Pool {
        double ratio_num = 0.0, ratio_den = 0.0;
        long long n = 0, cov95_n = 0, false_cert_n = 0;
    };
    std::vector<Pool> pools(3);

    int cell = 0;
    for (size_t mi = 0; mi < 3; ++mi) {
        const ModelSpec& spec = models[mi];
        for (double sigma_step : {0.0025, 0.005, 0.01, 0.02, 0.05}) {
            for (int n_views : {8, 16}) {
                std::vector<Vec2> truth;
                for (int i = 0; i < n_views; ++i) {
                    const double u = static_cast<double>(i) / (n_views - 1);
                    truth.push_back(Vec2{2.0 * u, 0.6 * std::sin(kPi * u)});
                }
                Rng rng(81000u + static_cast<unsigned>(cell++));
                std::vector<double> errs, zs;
                double pred_sum = 0.0;
                int pred_n = 0, false_cert_n = 0;
                for (int mc = 0; mc < n_mc; ++mc) {
                    RegistrationConfig rcfg;
                    rcfg.variance_model = spec.model;
                    WeightedWindowRegistration reg(rcfg);
                    Vec2 s_hat = truth.front();
                    double pose_var = 0.0;
                    for (int i = 0; i < n_views; ++i) {
                        if (i > 0) {
                            const Vec2 ds = truth[i] - truth[i - 1];
                            const double inc_sigma =
                                sigma_step * std::sqrt(steps_per_view);
                            s_hat += ds + Vec2{rng.gauss(inc_sigma),
                                               rng.gauss(inc_sigma)};
                            pose_var +=
                                steps_per_view * sigma_step * sigma_step;
                        }
                        RelayPacket p;
                        p.t = 0.05 * i;
                        const Vec2 lv = rotateT(theta, truth[i] - x_o);
                        const Vec2 lt = rotateT(theta, p_o - x_o);
                        p.r_v = lv.norm() + rng.gauss(sr);
                        p.beta_v = std::atan2(lv.y, lv.x) + rng.gauss(sb);
                        p.r_t = lt.norm() + rng.gauss(sr);
                        p.beta_t = std::atan2(lt.y, lt.x) + rng.gauss(sb);
                        p.sigma_r = sr;
                        p.sigma_beta = sb;
                        p.valid = true;
                        reg.addView(p.t, s_hat, p, pose_var);
                    }
                    double th, var, info, spread, corr;
                    if (!reg.solve(&th, &var, &info, &spread, &corr)) continue;
                    const double err = wrapAngle(th - theta);
                    const double z = err / std::sqrt(std::max(var, 1e-18));
                    const bool would_certify =
                        1.96 * std::sqrt(std::max(var, 0.0)) <= cert_on_rad;
                    const bool false_cert =
                        would_certify && std::fabs(err) > cert_on_rad;
                    errs.push_back(err);
                    zs.push_back(z);
                    pred_sum += var;
                    ++pred_n;
                    if (false_cert) ++false_cert_n;
                    ftr << spec.name << ',' << sigma_step << ',' << n_views
                        << ',' << mc << ',' << err << ',' << var << ',' << z
                        << ',' << (would_certify ? 1 : 0) << ','
                        << (false_cert ? 1 : 0) << '\n';
                }
                double mse = 0.0;
                int c90 = 0, c95 = 0, c99 = 0;
                for (size_t i = 0; i < errs.size(); ++i) {
                    mse += errs[i] * errs[i];
                    if (std::fabs(zs[i]) <= 1.645) ++c90;
                    if (std::fabs(zs[i]) <= 1.960) ++c95;
                    if (std::fabs(zs[i]) <= 2.576) ++c99;
                }
                const auto nz = std::max<size_t>(1, errs.size());
                mse /= nz;
                const double pred = pred_n ? pred_sum / pred_n : 0.0;
                const double ratio = pred > 0.0 ? mse / pred : 0.0;
                const double false_rate =
                    static_cast<double>(false_cert_n) / nz;
                f << spec.name << ',' << sigma_step << ',' << n_views << ','
                  << pred << ',' << mse << ',' << ratio << ','
                  << static_cast<double>(c90) / nz << ','
                  << static_cast<double>(c95) / nz << ','
                  << static_cast<double>(c99) / nz << ',' << false_rate
                  << ',' << errs.size() << '\n';
                std::printf(
                    "[S9] model=%-11s sigma=%.4f n=%d ratio=%.2f cov95=%.3f "
                    "false_cert=%.3f\n",
                    spec.name, sigma_step, n_views, ratio,
                    static_cast<double>(c95) / nz, false_rate);
                Pool& pool = pools[mi];
                pool.ratio_num += mse;
                pool.ratio_den += pred;
                pool.n += static_cast<long long>(errs.size());
                pool.cov95_n += c95;
                pool.false_cert_n += false_cert_n;
            }
        }
    }

    // Closed-loop success at the nominal condition, one cell per model
    // (matching S3 baselines: same seed base and geometry sampler).
    const MethodKind kinds[] = {MethodKind::kProposed,
                                MethodKind::kProposedDiagVar,
                                MethodKind::kProposedPacketOnly};
    const int n_closed_loop = 200;
    const unsigned seed_base = 10000;
    OdomErrorParams oe;
    oe.bias_vel = Vec2{0.01, -0.005};
    RelayErrorParams re;

    std::ofstream fs(std::string(kOut) +
                     "/s9_covariance_ablation_summary.csv");
    fs << "model,pooled_ratio,pooled_cov95,pooled_false_cert_rate,"
          "closed_loop_n,closed_loop_success_rate,closed_loop_wilson_lo,"
          "closed_loop_wilson_hi,closed_loop_median_rmse\n";
    for (size_t mi = 0; mi < 3; ++mi) {
        const Pool& pool = pools[mi];
        const double pooled_ratio =
            pool.ratio_den > 0.0 ? pool.ratio_num / pool.ratio_den : 0.0;
        const double pooled_cov95 =
            pool.n > 0 ? static_cast<double>(pool.cov95_n) / pool.n : 0.0;
        const double pooled_false_rate =
            pool.n > 0 ? static_cast<double>(pool.false_cert_n) / pool.n
                       : 0.0;

        int succ = 0;
        std::vector<double> rmse;
        for (int i = 0; i < n_closed_loop; ++i) {
            Rng grng(seed_base + i);
            const TrialGeometry geo = sampleGeometry(grng);
            const TrialMetrics m =
                runCampaignTrial(kinds[mi], geo, oe, re, seed_base + i);
            succ += m.success ? 1 : 0;
            rmse.push_back(m.station_rmse);
        }
        const WilsonInterval wi = wilson95(succ, n_closed_loop);
        fs << models[mi].name << ',' << pooled_ratio << ',' << pooled_cov95
           << ',' << pooled_false_rate << ',' << n_closed_loop << ','
           << static_cast<double>(succ) / n_closed_loop << ',' << wi.lo
           << ',' << wi.hi << ',' << medianOf(rmse) << '\n';
        std::printf(
            "[S9-SUMMARY] model=%-11s pooled_ratio=%.2f pooled_cov95=%.3f "
            "false_cert=%.3f closed_loop=%d/%d med_rmse=%.3f\n",
            models[mi].name, pooled_ratio, pooled_cov95, pooled_false_rate,
            succ, n_closed_loop, medianOf(rmse));
    }

    writeManifest(
        std::string(kOut) + "/s9_covariance_ablation_manifest.json",
        "s9_covariance_ablation", git_head, n_mc, 81000, 0.0,
        "\"measurement_noise\": {\"sigma_r_m\": 0.10, "
        "\"sigma_beta_rad\": 0.0175},\n  \"odometry_grid\": {"
        "\"sigma_xy_step_m\": [0.0025, 0.005, 0.01, 0.02, 0.05], "
        "\"n_views\": [8, 16], \"steps_per_view\": 5},\n  \"models\": "
        "[\"full\", \"diag\", \"packet_only\"],\n  \"closed_loop\": {"
        "\"n\": 200, \"seed_base\": 10000, \"cert_on_deg\": 10.0}");
}

// Study 3: independent relay range / bearing noise sweeps (proposed method).
void studyRelayNoise(int n_seeds, const std::string& git_head) {
    std::ofstream f(std::string(kOut) + "/s4_relay_noise.csv");
    f << "axis,sigma,n,success_rate,wilson_lo,wilson_hi,median_rmse,"
         "q90_rmse,median_theta_err\n";
    std::ofstream ftr(std::string(kOut) + "/s4_relay_noise_trials.csv");
    ftr << "axis,sigma,seed,success,time_to_goal,station_rmse,theta_err\n";
    OdomErrorParams oe;
    oe.bias_vel = Vec2{0.01, -0.005};
    const unsigned seed_base = 20000;
    auto runCell = [&](const char* axis, double sigma, RelayErrorParams re) {
        int succ = 0;
        std::vector<double> rmse, terr;
        for (int i = 0; i < n_seeds; ++i) {
            Rng grng(seed_base + i);
            const TrialGeometry geo = sampleGeometry(grng);
            const TrialMetrics m = runCampaignTrial(MethodKind::kProposed, geo,
                                                    oe, re, seed_base + i);
            ftr << axis << ',' << sigma << ',' << seed_base + i << ','
                << (m.success ? 1 : 0) << ',' << m.time_to_goal << ','
                << m.station_rmse << ',' << m.theta_err << '\n';
            succ += m.success ? 1 : 0;
            rmse.push_back(m.station_rmse);
            terr.push_back(m.theta_err);
        }
        const WilsonInterval wi = wilson95(succ, n_seeds);
        f << axis << ',' << sigma << ',' << n_seeds << ','
          << static_cast<double>(succ) / n_seeds << ',' << wi.lo << ','
          << wi.hi << ',' << medianOf(rmse) << ',' << quantileOf(rmse, 0.9)
          << ',' << medianOf(terr) << '\n';
        std::printf("[S4] %s sigma=%.4f success=%d/%d med_rmse=%.3f\n", axis,
                    sigma, succ, n_seeds, medianOf(rmse));
    };
    for (double sr : {0.025, 0.05, 0.10, 0.20, 0.40}) {
        RelayErrorParams re;
        re.sigma_r = sr;
        runCell("range", sr, re);
    }
    for (double sb_deg : {0.25, 0.5, 1.0, 2.0, 4.0}) {
        RelayErrorParams re;
        re.sigma_beta = sb_deg * kPi / 180.0;
        runCell("bearing_deg", sb_deg, re);
    }
    writeManifest(std::string(kOut) + "/s4_relay_noise_manifest.json",
                  "s4_relay_noise", git_head, n_seeds, seed_base, 150.0,
                  "\"sweep\": {\"sigma_r_m\": [0.025, 0.05, 0.10, "
                  "0.20, 0.40], \"sigma_beta_deg\": [0.25, 0.5, 1.0, "
                  "2.0, 4.0]},\n  \"odometry\": {\"sigma_xy_step_m\": "
                  "0.005, \"sigma_dtheta_step_rad\": 0.001, "
                  "\"bias_velocity_m_s\": [0.01, -0.005]}");
}

// Communication and gross-error stress campaign. These cases characterize
// the deployable pipeline rather than assuming ideal packet transport.
void studyCommunication(int n_seeds, const std::string& git_head) {
    std::ofstream f(std::string(kOut) + "/s8_communication.csv");
    f << "axis,value,n,success_rate,wilson_lo,wilson_hi,median_rmse,"
         "rmse_ci_lo,rmse_ci_hi,q90_rmse,median_theta_err,median_ttg\n";
    std::ofstream ftr(std::string(kOut) + "/s8_communication_trials.csv");
    ftr << "axis,value,seed,success,failure,time_to_goal,station_rmse,"
           "theta_err,adoptions\n";
    const unsigned seed_base = 60000;
    OdomErrorParams oe;
    oe.bias_vel = Vec2{0.01, -0.005};

    auto runCell = [&](const char* axis, double value, RelayErrorParams re) {
        int succ = 0;
        std::vector<double> rmse, terr, ttg;
        for (int i = 0; i < n_seeds; ++i) {
            Rng grng(seed_base + i);
            const TrialGeometry geo = sampleGeometry(grng);
            const TrialMetrics m = runCampaignTrial(
                MethodKind::kProposed, geo, oe, re, seed_base + i);
            ftr << axis << ',' << value << ',' << seed_base + i << ','
                << (m.success ? 1 : 0) << ',' << m.failure << ','
                << m.time_to_goal << ',' << m.station_rmse << ','
                << m.theta_err << ',' << m.adoptions << '\n';
            succ += m.success ? 1 : 0;
            rmse.push_back(m.station_rmse);
            terr.push_back(m.theta_err);
            if (m.success && m.time_to_goal >= 0.0)
                ttg.push_back(m.time_to_goal);
        }
        const WilsonInterval wi = wilson95(succ, n_seeds);
        double ci_lo, ci_hi;
        bootstrapMedianCi(rmse, &ci_lo, &ci_hi);
        f << axis << ',' << value << ',' << n_seeds << ','
          << static_cast<double>(succ) / n_seeds << ',' << wi.lo << ','
          << wi.hi << ',' << medianOf(rmse) << ',' << ci_lo << ',' << ci_hi
          << ',' << quantileOf(rmse, 0.9) << ',' << medianOf(terr) << ','
          << medianOf(ttg) << '\n';
        std::printf("[S8] %-12s=%.3f success=%d/%d med_rmse=%.3f\n",
                    axis, value, succ, n_seeds, medianOf(rmse));
    };

    for (double p : {0.0, 0.1, 0.3, 0.5}) {
        RelayErrorParams re;
        re.dropout = p;
        runCell("dropout", p, re);
    }
    for (double d : {0.0, 0.1, 0.25, 0.5}) {
        RelayErrorParams re;
        re.delay = d;
        runCell("delay_s", d, re);
    }
    for (double j : {0.0, 0.05, 0.1, 0.2}) {
        RelayErrorParams re;
        re.delay = 0.2;
        re.delay_jitter = j;
        runCell("jitter_s", j, re);
    }
    for (double p : {0.0, 0.01, 0.05, 0.10, 0.20}) {
        RelayErrorParams re;
        re.outlier_prob = p;
        runCell("outlier_prob", p, re);
    }
    writeManifest(
        std::string(kOut) + "/s8_communication_manifest.json",
        "s8_communication", git_head, n_seeds, seed_base, 150.0,
        "\"relay_noise\": {\"sigma_r_m\": 0.10, "
        "\"sigma_beta_rad\": 0.0175, \"outlier_scale_sigma\": 10.0},\n"
        "  \"sweep\": {\"dropout\": [0.0, 0.1, 0.3, 0.5], "
        "\"delay_s\": [0.0, 0.1, 0.25, 0.5], \"base_jitter_delay_s\": "
        "0.2, \"jitter_s\": [0.0, 0.05, 0.1, 0.2], "
        "\"outlier_probability\": [0.0, 0.01, 0.05, 0.10, 0.20]}");
}

// Study 6: odometry degradation and certificate operating boundary.
void studyOdometry(int n_seeds, const std::string& only_method,
                   const std::string& git_head,
                   const std::string& only_axis = "") {
    std::ofstream f(std::string(kOut) + "/s5_odometry_" +
                    (only_method.empty() ? "all" : only_method) +
                    (only_axis.empty() ? "" : "_" + only_axis) + ".csv");
    f << "axis,value,method,n,success_rate,wilson_lo,wilson_hi,median_rmse,"
         "q90_rmse,median_theta_err,median_path_err\n";
    std::ofstream ftr(std::string(kOut) + "/s5_odometry_trials_" +
                      (only_method.empty() ? "all" : only_method) +
                      (only_axis.empty() ? "" : "_" + only_axis) + ".csv");
    ftr << "axis,value,method,seed,success,failure,time_to_goal,station_rmse,"
           "theta_err,path_err,deadreck_err\n";
    const unsigned seed_base = 30000;
    RelayErrorParams re;
    const MethodKind kinds[] = {MethodKind::kProposed,
                                MethodKind::kProposedSmoother};
    auto runCell = [&](const char* axis, double value, OdomErrorParams oe) {
        if (!only_axis.empty() && only_axis != axis) return;
        for (MethodKind kind : kinds) {
            if (!only_method.empty() && only_method != methodName(kind))
                continue;
            int succ = 0;
            std::vector<double> rmse, terr, perr;
            for (int i = 0; i < n_seeds; ++i) {
                Rng grng(seed_base + i);
                const TrialGeometry geo = sampleGeometry(grng);
                const TrialMetrics m =
                    runCampaignTrial(kind, geo, oe, re, seed_base + i);
                ftr << axis << ',' << value << ',' << methodName(kind) << ','
                    << seed_base + i << ',' << (m.success ? 1 : 0) << ','
                    << m.failure << ',' << m.time_to_goal << ','
                    << m.station_rmse << ',' << m.theta_err << ','
                    << m.mean_path_err << ',' << m.deadreck_err << '\n';
                succ += m.success ? 1 : 0;
                rmse.push_back(m.station_rmse);
                terr.push_back(m.theta_err);
                perr.push_back(m.mean_path_err);
            }
            const WilsonInterval wi = wilson95(succ, n_seeds);
            f << axis << ',' << value << ',' << methodName(kind) << ','
              << n_seeds << ',' << static_cast<double>(succ) / n_seeds << ','
              << wi.lo << ',' << wi.hi << ',' << medianOf(rmse) << ','
              << quantileOf(rmse, 0.9) << ',' << medianOf(terr) << ','
              << medianOf(perr) << '\n';
            std::printf(
                "[S5] %s=%.4f %-18s success=%d/%d med_rmse=%.3f "
                "med_theta=%.4f med_path=%.3f\n",
                axis, value, methodName(kind), succ, n_seeds, medianOf(rmse),
                medianOf(terr), medianOf(perr));
        }
    };
    for (double sx : {0.005, 0.01, 0.02, 0.05}) {
        OdomErrorParams oe;
        oe.sigma_xy = sx;
        oe.bias_vel = Vec2{0.01, -0.005};
        runCell("sigma_xy", sx, oe);
    }
    for (double b : {0.0, 0.02, 0.05}) {
        OdomErrorParams oe;
        oe.bias_vel = Vec2{b, -0.5 * b};
        runCell("bias", b, oe);
    }
    for (double sc : {0.02, 0.05, 0.10}) {
        OdomErrorParams oe;
        oe.bias_vel = Vec2{0.01, -0.005};
        oe.scale_error = sc;
        runCell("scale", sc, oe);
    }
    for (double hd_deg : {0.1, 0.25, 0.5}) {
        OdomErrorParams oe;
        oe.bias_vel = Vec2{0.01, -0.005};
        oe.heading_drift = hd_deg * kPi / 180.0;
        runCell("heading_drift_deg_s", hd_deg, oe);
    }
    writeManifest(std::string(kOut) + "/s5_odometry_manifest_" +
                      (only_method.empty() ? "all" : only_method) +
                  (only_axis.empty() ? "" : "_" + only_axis) + ".json",
                  "s5_odometry", git_head, n_seeds, seed_base, 150.0,
                  "\"relay_noise\": {\"sigma_r_m\": 0.10, "
                  "\"sigma_beta_rad\": 0.0175},\n  \"sweep\": {"
                  "\"sigma_xy_step_m\": [0.005, 0.01, 0.02, 0.05], "
                  "\"bias_x_m_s\": [0.0, 0.02, 0.05], "
                  "\"scale_fraction\": [0.02, 0.05, 0.10], "
                  "\"heading_drift_deg_s\": [0.1, 0.25, 0.5]}");
}

// Study 9: long-horizon station keeping; drift non-accumulation evidence.
void studyDrift(int n_seeds, const std::string& git_head) {
    std::ofstream f(std::string(kOut) + "/s6_drift_summary.csv");
    f << "bias_m_s,n,success_rate,wilson_lo,wilson_hi,median_rmse,"
         "rmse_ci_lo,rmse_ci_hi,q90_rmse,median_deadreck_err\n";
    std::ofstream ftr(std::string(kOut) + "/s6_drift_trials.csv");
    ftr << "bias_m_s,seed,success,failure,time_to_goal,station_rmse,"
           "theta_err,deadreck_err\n";
    const unsigned seed_base = 40000;
    RelayErrorParams re;
    for (double b : {0.0, 0.01, 0.02, 0.05}) {
        OdomErrorParams oe;
        oe.bias_vel = Vec2{b, -0.5 * b};
        int succ = 0;
        std::vector<double> rmse, dr;
        for (int i = 0; i < n_seeds; ++i) {
            Rng grng(seed_base + i);
            const TrialGeometry geo = sampleGeometry(grng);
            const TrialMetrics m = runCampaignTrial(
                MethodKind::kProposed, geo, oe, re, seed_base + i, 600.0);
            ftr << b << ',' << seed_base + i << ',' << (m.success ? 1 : 0)
                << ',' << m.failure << ',' << m.time_to_goal << ','
                << m.station_rmse << ',' << m.theta_err << ','
                << m.deadreck_err << '\n';
            succ += m.success ? 1 : 0;
            rmse.push_back(m.station_rmse);
            dr.push_back(m.deadreck_err);
        }
        const WilsonInterval wi = wilson95(succ, n_seeds);
        double ci_lo, ci_hi;
        bootstrapMedianCi(rmse, &ci_lo, &ci_hi);
        f << b << ',' << n_seeds << ','
          << static_cast<double>(succ) / n_seeds << ',' << wi.lo << ','
          << wi.hi << ',' << medianOf(rmse) << ',' << ci_lo << ',' << ci_hi
          << ',' << quantileOf(rmse, 0.9) << ',' << medianOf(dr) << '\n';
        std::printf("[S6] bias=%.3f success=%d/%d med_rmse=%.3f "
                    "med_deadreck=%.1f\n",
                    b, succ, n_seeds, medianOf(rmse), medianOf(dr));
    }
    // Traced single run for the money plot.
    Rng grng(40042);
    const TrialGeometry geo = sampleGeometry(grng);
    OdomErrorParams oe;
    oe.bias_vel = Vec2{0.02, -0.01};
    WorldSE2 world(geo, oe, re, 0.01, 40042);
    SupervisorConfig scfg;
    RegistrationConfig rcfg;
    GfsPipeline pipe(scfg, rcfg);
    Vec2 s_dr{0, 0};
    std::ofstream ts(std::string(kOut) + "/s6_drift_timeseries.csv");
    ts << "t,dist,deadreck_err,mode\n";
    for (int k = 0; k <= 60000; ++k) {
        const double t = k * 0.01;
        if (k % 5 == 0) world.emitPacket(t);
        RelayPacket pkt;
        while (world.deliverDue(t, &pkt)) pipe.onPacket(pkt);
        const OdometrySample o = world.step(pipe.command(), t);
        s_dr += rotate(pipe.headingO(), o.dxy);
        pipe.onOdometry(o);
        if (k % 100 == 0)
            ts << t << ',' << world.distToTarget() << ','
               << (s_dr - world.sTrue()).norm() << ','
               << modeName(pipe.mode()) << '\n';
    }
    writeManifest(std::string(kOut) + "/s6_drift_manifest.json", "s6_drift",
                  git_head, n_seeds, seed_base, 600.0,
                  "\"relay_noise\": {\"sigma_r_m\": 0.10, "
                  "\"sigma_beta_rad\": 0.0175},\n  \"sweep\": {"
                  "\"body_bias_x_m_s\": [0.0, 0.01, 0.02, 0.05], "
                  "\"body_bias_y_ratio\": -0.5}");
}


// Disturbance-recovery study, redesigned per review 2026-08-16 #1.
// Scenarios:
//   transit : relay yaw steps at t=8 s (before arrival; median ttg 13.5 s).
//   station : yaw step at t=75 s during station keeping, then the TARGET
//             relocates by 2 m at t=90 s, so a stale calibration matters.
//   control : no disturbance at all -- measures false alarms (adoptions).
// Task success is restarted at the target event (t=90 s in station), while
// calibration detection and recovery are timed from the relay-yaw event.
// Changes already inside cert_off are classified as within_tolerance rather
// than counted as instantaneous recoveries.
void studyYawStep(int n_seeds, const std::string& git_head,
                  const std::string& scenario = "transit") {
    std::ofstream f(std::string(kOut) + "/s7_disturbance_" + scenario + ".csv");
    f << "scenario,variant,delta_deg,n,post_success_rate,post_wilson_lo,"
         "post_wilson_hi,adoption_rate,adoption_wilson_lo,adoption_wilson_hi,"
         "recovery_required_n,recovery_rate_required,recovery_wilson_lo,"
         "recovery_wilson_hi,within_tolerance_rate,median_detect_delay,"
         "median_recovery_time,median_post_time_to_goal,median_theta_err,"
         "median_station_rmse,median_adoptions\n";
    std::ofstream ftr(std::string(kOut) + "/s7_disturbance_" + scenario +
                      "_trials.csv");
    ftr << "scenario,variant,delta_deg,seed,post_success,post_time_to_goal,"
            "adoptions,detect_delay,recovery_required,within_tolerance,"
            "recovery_time,recovery_class,theta_err,station_rmse\n";
    const unsigned seed_base = 50000;
    OdomErrorParams oe;
    oe.bias_vel = Vec2{0.01, -0.005};
    RelayErrorParams re;
    struct Variant {
        const char* name;
        MethodKind kind;
    };
    const Variant variants[] = {
        {"supervised", MethodKind::kProposed},
        {"fixed_decay", MethodKind::kFixedDecayExc},
    };
    std::vector<double> deltas;
    if (scenario == "control") deltas = {0.0};
    else deltas = {20.0, 40.0, 80.0};

    for (double delta_deg : deltas) {
        for (const auto& v : variants) {
            int post_succ = 0, adopted = 0, recovered = 0;
            int recovery_required_n = 0, within_tolerance_n = 0;
            std::vector<double> dly, rec, pttg, terr, rmse, nad;
            for (int i = 0; i < n_seeds; ++i) {
                Rng grng(seed_base + i);
                const TrialGeometry geo = sampleGeometry(grng);
                TrialMetrics m;
                if (scenario == "transit") {
                    m = runCampaignTrial(v.kind, geo, oe, re, seed_base + i,
                                         200.0, 8.0,
                                         delta_deg * kPi / 180.0);
                } else if (scenario == "station") {
                    m = runCampaignTrial(v.kind, geo, oe, re, seed_base + i,
                                         250.0, 75.0,
                                         delta_deg * kPi / 180.0, 90.0,
                                         Vec2{2.0, -1.5});
                } else {  // control: no disturbance, count false adoptions
                    m = runCampaignTrial(v.kind, geo, oe, re, seed_base + i,
                                         200.0);
                    m.post_success = m.success;
                    m.post_time_to_goal = m.time_to_goal;
                }
                ftr << scenario << ',' << v.name << ',' << delta_deg << ','
                    << seed_base + i << ',' << (m.post_success ? 1 : 0) << ','
                    << m.post_time_to_goal << ',' << m.adoptions << ','
                    << m.detect_delay << ',' << (m.recovery_required ? 1 : 0)
                    << ',' << (m.within_tolerance ? 1 : 0) << ','
                    << m.recovery_time << ',' << m.recovery_class << ','
                    << m.theta_err << ',' << m.station_rmse << '\n';
                post_succ += m.post_success ? 1 : 0;
                adopted += m.adoptions > 0 ? 1 : 0;
                if (m.detect_delay >= 0.0) dly.push_back(m.detect_delay);
                if (m.recovery_required) {
                    ++recovery_required_n;
                }
                if (m.within_tolerance) ++within_tolerance_n;
                if (m.recovery_required && m.recovery_time >= 0.0) {
                    ++recovered;
                    rec.push_back(m.recovery_time);
                }
                if (m.post_success && m.post_time_to_goal >= 0.0)
                    pttg.push_back(m.post_time_to_goal);
                terr.push_back(m.theta_err);
                rmse.push_back(m.station_rmse);
                nad.push_back(m.adoptions);
            }
            const WilsonInterval post_wi = wilson95(post_succ, n_seeds);
            const WilsonInterval adopt_wi = wilson95(adopted, n_seeds);
            const WilsonInterval recovery_wi =
                wilson95(recovered, recovery_required_n);
            f << scenario << ',' << v.name << ',' << delta_deg << ','
              << n_seeds << ',' << static_cast<double>(post_succ) / n_seeds
              << ',' << post_wi.lo << ',' << post_wi.hi << ','
              << static_cast<double>(adopted) / n_seeds << ','
              << adopt_wi.lo << ',' << adopt_wi.hi << ','
              << recovery_required_n << ','
              << (recovery_required_n
                      ? static_cast<double>(recovered) / recovery_required_n
                      : 0.0)
              << ',' << recovery_wi.lo << ',' << recovery_wi.hi << ','
              << static_cast<double>(within_tolerance_n) / n_seeds << ','
              << medianOf(dly) << ',' << medianOf(rec) << ','
              << medianOf(pttg) << ',' << medianOf(terr) << ','
              << medianOf(rmse) << ',' << medianOf(nad) << '\n';
            std::printf(
                "[S7:%s] d=%.0f %-12s post_succ=%d/%d adopt=%.2f "
                "dly=%.1fs required=%d recov=%.2f rec_t=%.1fs th_err=%.3f\n",
                scenario.c_str(), delta_deg, v.name, post_succ, n_seeds,
                static_cast<double>(adopted) / n_seeds, medianOf(dly),
                recovery_required_n,
                recovery_required_n
                    ? static_cast<double>(recovered) / recovery_required_n
                    : 0.0,
                medianOf(rec),
                medianOf(terr));
        }
    }
    writeManifest(std::string(kOut) + "/s7_disturbance_" + scenario +
                      "_manifest.json",
                  "s7_disturbance_" + scenario, git_head, n_seeds, seed_base,
                  scenario == "station" ? 250.0 : 200.0,
                  "\"events\": {\"yaw_step_s\": " +
                      std::string(scenario == "transit" ? "8.0" :
                                  (scenario == "station" ? "75.0" : "null")) +
                      ", \"target_step_s\": " +
                      std::string(scenario == "station" ? "90.0" : "null") +
                      ", \"yaw_step_deg\": [20.0, 40.0, 80.0]},\n  "
                      "\"classification\": {\"recovery_threshold_rad\": " +
                      std::to_string(SupervisorConfig{}.cert_off_rad) +
                      ", \"post_success_starts_at_task_event\": true}");
}

}  // namespace

int main(int argc, char** argv) {
    const std::string study = argc > 1 ? argv[1] : "baselines";
    const int n = argc > 2 ? std::atoi(argv[2]) : 200;
    const std::string method = argc > 3 && std::strcmp(argv[3], "-") ? argv[3] : "";
    // The build embeds its source commit. A caller may provide the same full
    // hash or an unambiguous prefix, but cannot relabel a stale binary.
    const std::string compiled_head = GFS_GIT_HEAD;
    std::string git_head = argc > 4 ? argv[4] : compiled_head;
    auto is_hex = [](const std::string& h) {
        if (h.size() < 7 || h.size() > 40) return false;
        for (char c : h)
            if (!std::isxdigit(static_cast<unsigned char>(c))) return false;
        return true;
    };
    if (!is_hex(git_head) || !is_hex(compiled_head)) {
        std::fprintf(stderr,
                     "WARNING: build or requested git identity is unavailable; "
                     "results go to run_unspecified and must not be cited.\n");
        git_head = "unspecified";
    } else if (compiled_head.rfind(git_head, 0) != 0 &&
               git_head.rfind(compiled_head, 0) != 0) {
        std::fprintf(stderr,
                     "ERROR: requested git head %s does not match the binary "
                     "built from %s. Rebuild before running.\n",
                     git_head.c_str(), compiled_head.c_str());
        return 2;
    } else {
        git_head = compiled_head;
    }
    // Git working-tree cleanliness comes from the environment (set by the
    // driver script), avoiding positional-arg conflicts with study options.
    gitDirtyFlag() = GFS_CONFIGURE_DIRTY;
    if (const char* d = std::getenv("GFS_GIT_DIRTY")) gitDirtyFlag() = d;
    runIdentity() = git_head;
    if (const char* id = std::getenv("GFS_RUN_ID")) runIdentity() = id;
    for (char c : runIdentity()) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '-' &&
            c != '_') {
            std::fprintf(stderr, "ERROR: unsafe run identity '%s'.\n",
                         runIdentity().c_str());
            return 2;
        }
    }
    // All outputs land in a per-commit directory: stale files can never be
    // confused with current ones (P0 fix).
    g_out = std::string("results/campaign2027/offline/run_") + runIdentity();
    std::filesystem::create_directories(g_out);
    if (study == "baselines") studyBaselines(n, method, git_head);
    else if (study == "conditioning") studyConditioning(n, git_head);
    else if (study == "odometry_coverage")
        studyOdometryCoverage(n, git_head);
    else if (study == "relay_noise") studyRelayNoise(n, git_head);
    else if (study == "communication") studyCommunication(n, git_head);
    else if (study == "odometry")
        studyOdometry(n, method, git_head, argc > 5 ? argv[5] : "");
    else if (study == "drift") studyDrift(n, git_head);
    else if (study == "yaw_step")
        studyYawStep(n, git_head, method.empty() ? "transit" : method);
    else if (study == "covariance_ablation")
        studyCovarianceAblation(n, git_head);
    else {
        std::fprintf(stderr, "unknown study %s\n", study.c_str());
        return 1;
    }
    return 0;
}

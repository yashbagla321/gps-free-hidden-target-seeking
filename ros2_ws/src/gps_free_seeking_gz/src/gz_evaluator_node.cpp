// Ground-truth evaluator and run-integrity gate. The online estimator never
// receives the truth or diagnostics subscriptions owned by this node.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include "gps_free_seeking_gz/build_info.hpp"
#include "gps_free_seeking_msgs/msg/relay_diagnostics.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace {
double yawOf(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrap(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
}  // namespace

class GzEvaluatorNode final : public rclcpp::Node {
  public:
    GzEvaluatorNode() : Node("gz_evaluator_node") {
        target_x_ = declare_parameter<double>("target_x", 1.5);
        target_y_ = declare_parameter<double>("target_y", 9.0);
        target_step_time_ = declare_parameter<double>("target_step_time", -1.0);
        target_step_dx_ = declare_parameter<double>("target_step_dx", 0.0);
        target_step_dy_ = declare_parameter<double>("target_step_dy", 0.0);
        mission_s_ = declare_parameter<double>("mission_s", 90.0);
        station_window_s_ =
            declare_parameter<double>("station_window_s", 30.0);
        success_reach_ = declare_parameter<double>("success_reach", 0.25);
        success_hold_ = declare_parameter<double>("success_hold", 0.35);
        success_hold_s_ = declare_parameter<double>("success_hold_s", 10.0);
        seed_ = declare_parameter<int>("seed", 12);
        scenario_ = declare_parameter<std::string>("scenario", "manual");
        out_dir_ = declare_parameter<std::string>(
            "output_dir", "results/campaign2027/ros_gz/manual");
        out_name_ = declare_parameter<std::string>("output_name", "gz_run.csv");
        fail_if_exists_ =
            declare_parameter<bool>("fail_if_output_exists", true);

        if (mission_s_ <= 0.0 || station_window_s_ <= 0.0 ||
            success_hold_s_ <= 0.0)
            throw std::invalid_argument("mission and metric windows must be positive");
        if (scenario_.find_first_of(" \t\r\n=") != std::string::npos)
            throw std::invalid_argument("scenario must be one token");

        std::filesystem::create_directories(out_dir_);
        csv_path_ = out_dir_ + "/" + out_name_;
        summary_path_ = csv_path_ + ".summary";
        incomplete_path_ = csv_path_ + ".incomplete";
        if (fail_if_exists_ &&
            (std::filesystem::exists(csv_path_) ||
             std::filesystem::exists(summary_path_) ||
             std::filesystem::exists(incomplete_path_)))
            throw std::runtime_error("refusing to overwrite existing run output: " +
                                     csv_path_);
        std::filesystem::remove(summary_path_);
        std::filesystem::remove(incomplete_path_);
        ts_.open(csv_path_, std::ios::out | std::ios::trunc);
        if (!ts_) throw std::runtime_error("cannot open output: " + csv_path_);
        ts_ << "t,dist,mode,theta_ctrl,theta_var,e_hat_norm,retriggers,"
               "adoptions,odom_pos_err,odom_yaw_err,relay_sampled,"
               "relay_delivered,relay_dropped,relay_outliers,yaw_step_applied,"
               "yaw_step_elapsed,true_relay_yaw,true_theta,target_step_applied,"
               "target_step_elapsed,true_target_x,true_target_y\n";

        truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/gfs/truth_odom", rclcpp::QoS(20),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onTruth(*m); });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/model/gfs_vehicle/odometry", rclcpp::QoS(20),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                odom_ = *m;
                have_odom_ = true;
            });
        status_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gfs/status", rclcpp::QoS(20),
            [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) {
                if (m->data.size() >= 6) status_ = *m;
            });
        diag_sub_ =
            create_subscription<gps_free_seeking_msgs::msg::RelayDiagnostics>(
                "/relay/diagnostics", rclcpp::QoS(20),
                [this](gps_free_seeking_msgs::msg::RelayDiagnostics::ConstSharedPtr m) {
                    diagnostics_ = *m;
                    have_diagnostics_ = true;
                });
    }

    ~GzEvaluatorNode() override {
        if (!finished_) {
            ts_.close();
            writeIncomplete("node_stopped_before_mission_end");
        }
    }

  private:
    void onTruth(const nav_msgs::msg::Odometry& m) {
        const double t = rclcpp::Time(m.header.stamp).seconds();
        if (start_t_ < 0.0) start_t_ = t;
        elapsed_ = t - start_t_;

        double tx = target_x_, ty = target_y_;
        if (have_diagnostics_) {
            tx = diagnostics_.true_target_x;
            ty = diagnostics_.true_target_y;
        } else if (target_step_time_ > 0.0 && elapsed_ >= target_step_time_) {
            tx += target_step_dx_;
            ty += target_step_dy_;
        }
        const double dist = std::hypot(m.pose.pose.position.x - tx,
                                       m.pose.pose.position.y - ty);
        if (dist <= success_reach_ && first_reach_ < 0.0)
            first_reach_ = elapsed_;
        if (dist <= success_hold_) {
            if (hold_start_ < 0.0 && first_reach_ >= 0.0)
                hold_start_ = elapsed_;
            if (hold_start_ >= 0.0 && elapsed_ - hold_start_ >= success_hold_s_)
                success_ = true;
        } else {
            hold_start_ = -1.0;
        }

        double odom_pos_err = -1.0, odom_yaw_err = -1.0;
        if (have_odom_) {
            const double ox = odom_.pose.pose.position.x;
            const double oy = odom_.pose.pose.position.y;
            const double oyaw = yawOf(odom_.pose.pose.orientation);
            if (!have_odom_alignment_) {
                odom_yaw_offset_ = wrap(yawOf(m.pose.pose.orientation) - oyaw);
                const double c = std::cos(odom_yaw_offset_);
                const double s = std::sin(odom_yaw_offset_);
                odom_tx_ = m.pose.pose.position.x - (c * ox - s * oy);
                odom_ty_ = m.pose.pose.position.y - (s * ox + c * oy);
                have_odom_alignment_ = true;
            }
            const double c = std::cos(odom_yaw_offset_);
            const double s = std::sin(odom_yaw_offset_);
            const double aligned_x = odom_tx_ + c * ox - s * oy;
            const double aligned_y = odom_ty_ + s * ox + c * oy;
            odom_pos_err = std::hypot(
                aligned_x - m.pose.pose.position.x,
                aligned_y - m.pose.pose.position.y);
            odom_yaw_err = std::fabs(wrap(
                oyaw + odom_yaw_offset_ - yawOf(m.pose.pose.orientation)));
        }
        if (elapsed_ >= mission_s_ - station_window_s_) {
            tail2_ += dist * dist;
            ++tail_n_;
            if (odom_pos_err >= 0.0) {
                odom_pos_tail2_ += odom_pos_err * odom_pos_err;
                odom_yaw_tail2_ += odom_yaw_err * odom_yaw_err;
                ++odom_tail_n_;
            }
        }

        if (++decim_ % 10 == 0)
            writeCsvRow(dist, odom_pos_err, odom_yaw_err);
        if (elapsed_ >= mission_s_) {
            finishComplete();
            rclcpp::shutdown();
        }
    }

    void writeCsvRow(double dist, double odom_pos_err, double odom_yaw_err) {
        const auto& d = status_.data;
        ts_ << elapsed_ << ',' << dist << ','
            << (d.size() >= 6 ? d[0] : -1.0) << ','
            << (d.size() >= 6 ? d[1] : 0.0) << ','
            << (d.size() >= 6 ? d[2] : 0.0) << ','
            << (d.size() >= 6 ? d[3] : 0.0) << ','
            << (d.size() >= 6 ? d[4] : 0.0) << ','
            << (d.size() >= 6 ? d[5] : 0.0) << ','
            << odom_pos_err << ',' << odom_yaw_err << ','
            << (have_diagnostics_ ? diagnostics_.sampled : 0) << ','
            << (have_diagnostics_ ? diagnostics_.delivered : 0) << ','
            << (have_diagnostics_ ? diagnostics_.dropped : 0) << ','
            << (have_diagnostics_ ? diagnostics_.outliers : 0) << ','
            << (have_diagnostics_ && diagnostics_.yaw_step_applied ? 1 : 0)
            << ','
            << (have_diagnostics_ ? diagnostics_.yaw_step_elapsed : -1.0) << ','
            << (have_diagnostics_ ? diagnostics_.true_relay_yaw : 0.0) << ','
            << trueThetaFromDiagnostics() << ','
            << (have_diagnostics_ && diagnostics_.target_step_applied ? 1 : 0)
            << ','
            << (have_diagnostics_ ? diagnostics_.target_step_elapsed : -1.0)
            << ',' << txFromDiagnostics() << ',' << tyFromDiagnostics() << '\n';
        ++csv_rows_;
    }

    double txFromDiagnostics() const {
        return have_diagnostics_ ? diagnostics_.true_target_x : target_x_;
    }
    double tyFromDiagnostics() const {
        return have_diagnostics_ ? diagnostics_.true_target_y : target_y_;
    }
    // The pipeline's theta_ctrl estimates the gauge-quotient invariant
    // theta = psi - gamma (relay yaw minus the vehicle's unknown odometry-
    // frame heading offset), NOT the raw relay yaw psi carried in
    // true_relay_yaw. gamma is exactly the constant odom-to-world yaw
    // offset already computed once at the first synchronized truth/odom
    // sample (odom_yaw_offset_), so the correct ground truth for theta_ctrl
    // is true_relay_yaw - odom_yaw_offset_, wrapped. Comparing theta_ctrl
    // directly to true_relay_yaw compares two quantities that differ by
    // this constant (about 0.4 rad / 23 deg here) and would make any
    // fixed-tolerance recovery check spuriously fail.
    double trueThetaFromDiagnostics() const {
        if (!have_diagnostics_ || !have_odom_alignment_) return 0.0;
        return wrap(diagnostics_.true_relay_yaw - odom_yaw_offset_);
    }

    void finishComplete() {
        if (finished_) return;
        if (elapsed_ + 0.05 < mission_s_ || tail_n_ == 0 || csv_rows_ == 0 ||
            !have_diagnostics_ || !have_odom_) {
            ts_.close();
            writeIncomplete("mission_integrity_check_failed");
            finished_ = true;
            return;
        }
        ts_.close();
        const double rmse = std::sqrt(tail2_ / tail_n_);
        const double odom_pos_rmse = odom_tail_n_ > 0
            ? std::sqrt(odom_pos_tail2_ / odom_tail_n_) : -1.0;
        const double odom_yaw_rmse = odom_tail_n_ > 0
            ? std::sqrt(odom_yaw_tail2_ / odom_tail_n_) : -1.0;
        const std::string tmp = summary_path_ + ".tmp";
        {
            std::ofstream sum(tmp, std::ios::out | std::ios::trunc);
            if (!sum) throw std::runtime_error("cannot write summary: " + tmp);
            sum << "complete=1 scenario=" << scenario_ << " seed=" << seed_
                << " build_commit=" << gfs_gz_build::kGitCommit
                << " build_dirty=" << (gfs_gz_build::kDirty ? 1 : 0)
                << " expected_duration=" << mission_s_
                << " elapsed=" << elapsed_ << " csv_rows=" << csv_rows_
                << " success=" << (success_ ? 1 : 0)
                << " time_to_reach=" << first_reach_
                << " station_rmse=" << rmse
                << " odom_position_rmse=" << odom_pos_rmse
                << " odom_yaw_rmse=" << odom_yaw_rmse
                << " relay_sampled=" << diagnostics_.sampled
                << " relay_delivered=" << diagnostics_.delivered
                << " relay_dropped=" << diagnostics_.dropped
                << " relay_outliers=" << diagnostics_.outliers
                << " yaw_step_applied="
                << (diagnostics_.yaw_step_applied ? 1 : 0)
                << " yaw_step_elapsed=" << diagnostics_.yaw_step_elapsed
                << " target_step_applied="
                << (diagnostics_.target_step_applied ? 1 : 0)
                << " target_step_elapsed=" << diagnostics_.target_step_elapsed
                << '\n';
        }
        std::filesystem::rename(tmp, summary_path_);
        finished_ = true;
        RCLCPP_INFO(get_logger(),
                    "mission complete: success=%d reach=%.2fs rmse=%.3fm "
                    "odom_rmse=%.3fm",
                    success_ ? 1 : 0, first_reach_, rmse, odom_pos_rmse);
    }

    void writeIncomplete(const std::string& reason) {
        if (std::filesystem::exists(summary_path_)) return;
        const std::string tmp = incomplete_path_ + ".tmp";
        {
            std::ofstream out(tmp, std::ios::out | std::ios::trunc);
            out << "complete=0 reason=" << reason
                << " scenario=" << scenario_ << " seed=" << seed_
                << " build_commit=" << gfs_gz_build::kGitCommit
                << " build_dirty=" << (gfs_gz_build::kDirty ? 1 : 0)
                << " expected_duration=" << mission_s_
                << " elapsed=" << elapsed_ << " csv_rows=" << csv_rows_
                << '\n';
        }
        std::filesystem::rename(tmp, incomplete_path_);
    }

    double target_x_, target_y_, target_step_time_, target_step_dx_,
        target_step_dy_, mission_s_, station_window_s_, success_reach_,
        success_hold_, success_hold_s_;
    int seed_;
    std::string scenario_, out_dir_, out_name_, csv_path_, summary_path_,
        incomplete_path_;
    bool fail_if_exists_;
    std::ofstream ts_;
    std_msgs::msg::Float64MultiArray status_;
    nav_msgs::msg::Odometry odom_;
    gps_free_seeking_msgs::msg::RelayDiagnostics diagnostics_;
    double start_t_ = -1.0, elapsed_ = -1.0, first_reach_ = -1.0,
           hold_start_ = -1.0;
    double odom_yaw_offset_ = 0.0, odom_tx_ = 0.0, odom_ty_ = 0.0;
    double tail2_ = 0.0, odom_pos_tail2_ = 0.0, odom_yaw_tail2_ = 0.0;
    int tail_n_ = 0, odom_tail_n_ = 0, decim_ = 0, csv_rows_ = 0;
    bool success_ = false, finished_ = false, have_odom_ = false,
         have_diagnostics_ = false, have_odom_alignment_ = false;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_,
        odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        status_sub_;
    rclcpp::Subscription<gps_free_seeking_msgs::msg::RelayDiagnostics>::SharedPtr
        diag_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GzEvaluatorNode>());
    rclcpp::shutdown();
    return 0;
}

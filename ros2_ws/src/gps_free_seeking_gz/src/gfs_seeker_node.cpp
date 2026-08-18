// GPS-free seeker node: the vehicle-side pipeline on real ROS interfaces.
//
// GROUND-TRUTH ISOLATION (auditable): this node subscribes to exactly two
// data topics, the physics wheel-odometry stream /odom (from the Gazebo
// differential-drive system, i.e., integrated wheel motion with slip and
// inertia, NOT ground truth) and the asynchronous relay packets
// /relay/packet. It publishes /cmd_vel and diagnostics. It never sees any
// world-frame quantity: `ros2 node info /gfs_seeker_node` is the audit.
//
// Algorithm per odometry message: convert the pose delta between
// consecutive odometry poses into a body-frame SE(2) increment and feed the
// shared GfsPipeline (the same header-only core as the offline campaign).
// Per relay packet: feed the pipeline (registration, certificate,
// control-path update, delay compensation) and publish the resulting
// saturated unicycle command.

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "gps_free_seeking/Supervisor.hpp"
#include "gps_free_seeking_msgs/msg/relay_packet.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace {
double yawOf(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

class GfsSeekerNode final : public rclcpp::Node {
  public:
    GfsSeekerNode() : Node("gfs_seeker_node") {
        gfs::SupervisorConfig scfg;
        scfg.cert_on_rad =
            declare_parameter<double>("cert_on_deg", 10.0) * M_PI / 180.0;
        scfg.cert_off_rad =
            declare_parameter<double>("cert_off_deg", 15.0) * M_PI / 180.0;
        scfg.min_correlation =
            declare_parameter<double>("min_correlation", 0.5);
        scfg.min_window_packets =
            declare_parameter<int>("min_window_packets", 8);
        scfg.consecutive_windows =
            declare_parameter<int>("consecutive_windows", 3);
        scfg.min_dwell = declare_parameter<double>("min_dwell_s", 1.0);
        scfg.v_excite = declare_parameter<double>("v_excite", 0.6);
        scfg.omega_excite = declare_parameter<double>("omega_excite", 0.45);
        scfg.k_p = declare_parameter<double>("k_p", 0.8);
        scfg.k_omega = declare_parameter<double>("k_omega", 2.0);
        scfg.v_max = declare_parameter<double>("v_max", 1.0);
        scfg.omega_max = declare_parameter<double>("omega_max", 1.2);
        scfg.maintain_enter = declare_parameter<double>("maintain_enter", 0.20);
        scfg.maintain_exit = declare_parameter<double>("maintain_exit", 0.30);
        scfg.k_e = declare_parameter<double>("k_e", 2.0);
        scfg.theta_drift_rate =
            declare_parameter<double>("theta_drift_rate", 0.002);
        scfg.change_chi_thresh =
            declare_parameter<double>("change_chi_thresh", 9.0);
        scfg.change_adopt_after =
            declare_parameter<int>("change_adopt_after", 10);
        scfg.change_min_duration =
            declare_parameter<double>("change_min_duration_s", 4.0);
        scfg.change_min_interval =
            declare_parameter<double>("change_min_interval_s", 2.0);
        gfs::RegistrationConfig rcfg;
        rcfg.max_packets =
            static_cast<int>(declare_parameter<int>("window_packets", 64));
        rcfg.max_age = declare_parameter<double>("window_age_s", 4.0);
        rcfg.min_packets = declare_parameter<int>("min_packets", 2);
        rcfg.min_spread = declare_parameter<double>("min_spread", 1e-4);
        // Continuous-time random-walk intensities make the propagated
        // uncertainty invariant to the odometry publication rate. The
        // offline 0.005 m and 0.001 rad increments at 100 Hz correspond to
        // 0.05 m/sqrt(s) and 0.01 rad/sqrt(s), respectively.
        sigma_xy_density_ = declare_parameter<double>(
            "odom_sigma_xy_per_sqrt_s", 0.05);
        sigma_dth_density_ = declare_parameter<double>(
            "odom_sigma_dtheta_per_sqrt_s", 0.01);
        pipe_ = std::make_unique<gfs::GfsPipeline>(scfg, rcfg);

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(10));
        status_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gfs/status", rclcpp::QoS(10));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(50),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                onOdom(*m);
            });
        pkt_sub_ = create_subscription<gps_free_seeking_msgs::msg::RelayPacket>(
            "/relay/packet", rclcpp::QoS(50),
            [this](gps_free_seeking_msgs::msg::RelayPacket::ConstSharedPtr m) {
                onPacket(*m);
            });
        // Command at a steady rate so the vehicle keeps its last command
        // between packets (matching the offline campaign's control cadence).
        // Sim-time timer (follows /clock at any real-time factor).
        cmd_timer_ = create_timer(std::chrono::milliseconds(20),
                                  [this]() { publishCommand(); });
    }

  private:
    // Wheel-odometry pose stream -> body-frame SE(2) increments for the
    // pipeline. The very first message only initializes the reference.
    void onOdom(const nav_msgs::msg::Odometry& m) {
        const double t = rclcpp::Time(m.header.stamp).seconds();
        const double x = m.pose.pose.position.x;
        const double y = m.pose.pose.position.y;
        const double yaw = yawOf(m.pose.pose.orientation);
        if (have_prev_) {
            const double c = std::cos(prev_yaw_), s = std::sin(prev_yaw_);
            gfs::OdometrySample o;
            o.t = t;
            // World-of-odom delta rotated into the previous body frame.
            o.dxy = gfs::Vec2{c * (x - prev_x_) + s * (y - prev_y_),
                              -s * (x - prev_x_) + c * (y - prev_y_)};
            o.dtheta = std::atan2(std::sin(yaw - prev_yaw_),
                                  std::cos(yaw - prev_yaw_));
            // Physical-plausibility gate: an increment implying far more
            // than v_max between consecutive 50 Hz messages is a glitch
            // (sensor fault, replay, or a foreign pose stream), never real
            // motion. Reject it and re-anchor instead of poisoning the
            // task filter.
            const double dt_msg = std::max(t - prev_t_, 1e-3);
            o.sigma_xy = sigma_xy_density_ * std::sqrt(dt_msg);
            o.sigma_dtheta = sigma_dth_density_ * std::sqrt(dt_msg);
            if (o.dxy.norm() <= 5.0 * 1.5 * dt_msg + 0.05 &&
                std::fabs(o.dtheta) <= 5.0 * 2.0 * dt_msg + 0.1) {
                pipe_->onOdometry(o);
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "rejected implausible odometry "
                                     "increment |dxy|=%.2f m in %.3f s",
                                     o.dxy.norm(), dt_msg);
            }
        }
        prev_x_ = x;
        prev_y_ = y;
        prev_yaw_ = yaw;
        prev_t_ = t;
        have_prev_ = true;
    }

    void onPacket(const gps_free_seeking_msgs::msg::RelayPacket& m) {
        gfs::RelayPacket p;
        p.t = rclcpp::Time(m.stamp).seconds();
        p.r_v = m.r_v;
        p.beta_v = m.beta_v;
        p.r_t = m.r_t;
        p.beta_t = m.beta_t;
        p.sigma_r = m.sigma_r;
        p.sigma_beta = m.sigma_beta;
        p.valid = true;
        pipe_->onPacket(p);

        std_msgs::msg::Float64MultiArray st;
        const auto& est = pipe_->estimate();
        st.data = {static_cast<double>(static_cast<int>(pipe_->mode())),
                   pipe_->thetaCtrl(),
                   est.theta_variance,
                   est.e_hat.norm(),
                   static_cast<double>(pipe_->retriggers()),
                   static_cast<double>(pipe_->adoptions())};
        status_pub_->publish(st);
    }

    void publishCommand() {
        const auto& c = pipe_->command();
        geometry_msgs::msg::Twist tw;
        tw.linear.x = c.v;
        tw.angular.z = c.omega;
        cmd_pub_->publish(tw);
    }

    std::unique_ptr<gfs::GfsPipeline> pipe_;
    double sigma_xy_density_, sigma_dth_density_;
    double prev_x_ = 0.0, prev_y_ = 0.0, prev_yaw_ = 0.0, prev_t_ = 0.0;
    bool have_prev_ = false;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<gps_free_seeking_msgs::msg::RelayPacket>::SharedPtr
        pkt_sub_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GfsSeekerNode>());
    rclcpp::shutdown();
    return 0;
}

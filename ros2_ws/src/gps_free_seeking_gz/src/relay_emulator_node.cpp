// Relay emulator: the ONLY runtime node allowed to read ground truth.
//
// Reads the vehicle's true pose from the dedicated truth odometry channel,
// forms native range-bearing packets to the vehicle and the hidden target in
// the relay's own (unknown to the estimator) frame, corrupts them with
// noise and per-packet outliers, and delivers them ASYNCHRONOUSLY through
// the ROS graph with configurable dropout, fixed delay, and jitter: packets
// carry their measurement stamp, delivery happens when a sim-time timer
// finds them due. A relay yaw step and a target relocation can be injected
// at parameterized times, so the disturbance reaches the estimator only
// through the ROS interfaces, exactly as it would from real hardware.

#include <cmath>
#include <deque>
#include <memory>
#include <random>

#include "geometry_msgs/msg/pose.hpp"
#include "gps_free_seeking_msgs/msg/relay_diagnostics.hpp"
#include "gps_free_seeking_msgs/msg/relay_packet.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
double wrap(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
}  // namespace

class RelayEmulatorNode final : public rclcpp::Node {
  public:
    RelayEmulatorNode() : Node("relay_emulator_node") {
        relay_x_ = declare_parameter<double>("relay_x", -6.0);
        relay_y_ = declare_parameter<double>("relay_y", -1.5);
        relay_yaw_ = declare_parameter<double>("relay_yaw", 2.2);
        target_x_ = declare_parameter<double>("target_x", 1.5);
        target_y_ = declare_parameter<double>("target_y", 9.0);
        rate_hz_ = declare_parameter<double>("rate_hz", 20.0);
        sigma_r_ = declare_parameter<double>("sigma_r", 0.10);
        sigma_beta_ = declare_parameter<double>("sigma_beta", 0.0175);
        dropout_ = declare_parameter<double>("dropout", 0.0);
        delay_s_ = declare_parameter<double>("delay_s", 0.0);
        delay_jitter_s_ = declare_parameter<double>("delay_jitter_s", 0.0);
        outlier_prob_ = declare_parameter<double>("outlier_prob", 0.0);
        outlier_scale_ = declare_parameter<double>("outlier_scale", 10.0);
        yaw_step_time_ = declare_parameter<double>("yaw_step_time", -1.0);
        yaw_step_deg_ = declare_parameter<double>("yaw_step_deg", 0.0);
        target_step_time_ = declare_parameter<double>("target_step_time", -1.0);
        target_step_dx_ = declare_parameter<double>("target_step_dx", 0.0);
        target_step_dy_ = declare_parameter<double>("target_step_dy", 0.0);
        rng_.seed(static_cast<unsigned>(declare_parameter<int>("seed", 12)));

        pub_ = create_publisher<gps_free_seeking_msgs::msg::RelayPacket>(
            "/relay/packet", rclcpp::QoS(50));
        diag_pub_ =
            create_publisher<gps_free_seeking_msgs::msg::RelayDiagnostics>(
                "/relay/diagnostics", rclcpp::QoS(20));
        truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/gfs/truth_odom", rclcpp::QoS(20),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                truth_ = *m;
                have_truth_ = true;
            });
        // Measurement clock: sample and enqueue at rate_hz on SIM time.
        // Sim-time timers: cadence must follow /clock, not the wall clock,
        // so batches behave identically at any real-time factor.
        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        sample_timer_ = create_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { sample(); });
        // Delivery clock: pop due packets (asynchronous delivery path).
        deliver_timer_ = create_timer(std::chrono::milliseconds(5),
                                      [this]() { deliver(); });
    }

  private:
    double gauss(double s) {
        return s > 0.0 ? std::normal_distribution<double>(0.0, s)(rng_) : 0.0;
    }
    double uni() {
        return std::uniform_real_distribution<double>(0.0, 1.0)(rng_);
    }

    // Form one packet from the CURRENT true pose; apply the configured
    // disturbances and channel corruption; enqueue with a delivery time.
    void sample() {
        if (!have_truth_) return;
        const double now = get_clock()->now().seconds();
        if (start_t_ < 0.0) start_t_ = now;
        const double elapsed = now - start_t_;

        const double psi =
            (yaw_step_time_ > 0.0 && elapsed >= yaw_step_time_)
                ? relay_yaw_ + yaw_step_deg_ * M_PI / 180.0
                : relay_yaw_;
        double tx = target_x_, ty = target_y_;
        if (target_step_time_ > 0.0 && elapsed >= target_step_time_) {
            tx += target_step_dx_;
            ty += target_step_dy_;
        }
        const bool yaw_applied = yaw_step_time_ > 0.0 &&
                                 elapsed >= yaw_step_time_;
        const bool target_applied = target_step_time_ > 0.0 &&
                                    elapsed >= target_step_time_;
        if (yaw_applied && !yaw_step_applied_) yaw_step_elapsed_ = elapsed;
        if (target_applied && !target_step_applied_)
            target_step_elapsed_ = elapsed;
        yaw_step_applied_ = yaw_applied;
        target_step_applied_ = target_applied;
        true_relay_yaw_ = wrap(psi);
        true_target_x_ = tx;
        true_target_y_ = ty;
        ++sampled_;
        if (dropout_ > 0.0 && uni() < dropout_) {
            ++dropped_;
            publishDiagnostics(now, elapsed);
            return;
        }
        const double qx = truth_.pose.pose.position.x;
        const double qy = truth_.pose.pose.position.y;
        const double c = std::cos(psi), s = std::sin(psi);
        // Relay-frame vectors l = R(psi)^T (p - x).
        const double vx = c * (qx - relay_x_) + s * (qy - relay_y_);
        const double vy = -s * (qx - relay_x_) + c * (qy - relay_y_);
        const double wx = c * (tx - relay_x_) + s * (ty - relay_y_);
        const double wy = -s * (tx - relay_x_) + c * (ty - relay_y_);

        const bool outlier = outlier_prob_ > 0.0 && uni() < outlier_prob_;
        if (outlier) ++outliers_;
        const double boost = outlier ? outlier_scale_ : 1.0;
        gps_free_seeking_msgs::msg::RelayPacket m;
        m.stamp = get_clock()->now();
        m.r_v = std::hypot(vx, vy) + gauss(sigma_r_) * boost;
        m.beta_v = wrap(std::atan2(vy, vx) + gauss(sigma_beta_) * boost);
        m.r_t = std::hypot(wx, wy) + gauss(sigma_r_) * boost;
        m.beta_t = wrap(std::atan2(wy, wx) + gauss(sigma_beta_) * boost);
        m.sigma_r = sigma_r_;
        m.sigma_beta = sigma_beta_;

        double delay = delay_s_;
        if (delay_jitter_s_ > 0.0)
            delay += (2.0 * uni() - 1.0) * delay_jitter_s_;
        queue_.push_back({now + std::max(0.0, delay), m});
        publishDiagnostics(now, elapsed);
    }

    void deliver() {
        const double now = get_clock()->now().seconds();
        for (auto it = queue_.begin(); it != queue_.end();) {
            if (it->deliver_at <= now) {
                pub_->publish(it->pkt);
                ++delivered_;
                it = queue_.erase(it);
            } else {
                ++it;
            }
        }
    }

    void publishDiagnostics(double, double elapsed) {
        gps_free_seeking_msgs::msg::RelayDiagnostics d;
        d.stamp = get_clock()->now();
        d.elapsed = elapsed;
        d.sampled = sampled_;
        d.delivered = delivered_;
        d.dropped = dropped_;
        d.outliers = outliers_;
        d.yaw_step_applied = yaw_step_applied_;
        d.yaw_step_elapsed = yaw_step_elapsed_;
        d.true_relay_yaw = true_relay_yaw_;
        d.target_step_applied = target_step_applied_;
        d.target_step_elapsed = target_step_elapsed_;
        d.true_target_x = true_target_x_;
        d.true_target_y = true_target_y_;
        diag_pub_->publish(d);
    }

    struct Pending {
        double deliver_at;
        gps_free_seeking_msgs::msg::RelayPacket pkt;
    };

    double relay_x_, relay_y_, relay_yaw_, target_x_, target_y_;
    double rate_hz_, sigma_r_, sigma_beta_, dropout_, delay_s_,
        delay_jitter_s_, outlier_prob_, outlier_scale_;
    double yaw_step_time_, yaw_step_deg_, target_step_time_, target_step_dx_,
        target_step_dy_;
    std::mt19937 rng_;
    nav_msgs::msg::Odometry truth_;
    bool have_truth_ = false;
    double start_t_ = -1.0;
    double yaw_step_elapsed_ = -1.0, target_step_elapsed_ = -1.0;
    double true_relay_yaw_ = 0.0, true_target_x_ = 0.0,
           true_target_y_ = 0.0;
    uint64_t sampled_ = 0, delivered_ = 0, dropped_ = 0, outliers_ = 0;
    bool yaw_step_applied_ = false, target_step_applied_ = false;
    std::deque<Pending> queue_;
    rclcpp::Publisher<gps_free_seeking_msgs::msg::RelayPacket>::SharedPtr pub_;
    rclcpp::Publisher<gps_free_seeking_msgs::msg::RelayDiagnostics>::SharedPtr
        diag_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
    rclcpp::TimerBase::SharedPtr sample_timer_, deliver_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelayEmulatorNode>());
    rclcpp::shutdown();
    return 0;
}

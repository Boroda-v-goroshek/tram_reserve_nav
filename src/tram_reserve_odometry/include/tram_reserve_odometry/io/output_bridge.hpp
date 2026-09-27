// Выходной мост: Estimate -> ROS-сообщения.
//   /result/velocity : tram_vehicle_msgs/msg/VelocitySensor  (velocity, м/с)
//   /result/position : nav_msgs/msg/Odometry                (pose.pose.position)
//   header.stamp     : время входа (bag)
//   /debug/state, /debug/slip, /diagnostics — отладка.

#ifndef TRAM_RESERVE_ODOMETRY__IO__OUTPUT_BRIDGE_HPP_
#define TRAM_RESERVE_ODOMETRY__IO__OUTPUT_BRIDGE_HPP_

#include <array>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tram_vehicle_msgs/msg/velocity_sensor.hpp>

#include "tram_reserve_odometry/core/types.hpp"

namespace tram
{

class OutputBridge
{
public:
  struct Config
  {
    std::string velocity_topic = "/result/velocity";
    std::string position_topic = "/result/position";
    std::string state_topic = "/debug/state";
    std::string slip_topic = "/debug/slip";
    std::string diagnostics_topic = "/diagnostics";
    std::string frame_id = "map";
    std::string child_frame_id = "tram_base_link";
    bool publish_debug = true;
  };

  explicit OutputBridge(rclcpp::Node * node, const Config & cfg)
  : node_(node), cfg_(cfg)
  {
    pub_velocity_ = node_->create_publisher<tram_vehicle_msgs::msg::VelocitySensor>(
      cfg_.velocity_topic, rclcpp::SystemDefaultsQoS());
    pub_position_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      cfg_.position_topic, rclcpp::SystemDefaultsQoS());
    pub_state_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      cfg_.state_topic, rclcpp::SystemDefaultsQoS());
    pub_slip_ = node_->create_publisher<std_msgs::msg::Float64>(
      cfg_.slip_topic, rclcpp::SystemDefaultsQoS());
    pub_diagnostics_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      cfg_.diagnostics_topic, rclcpp::SystemDefaultsQoS());
  }

  /// Опубликовать оценку. \p latency_ms — задержка вход -> выход;
  /// \p with_position = false — позиция не определена, публикуется только скорость.
  void publish(const Estimate & est, double latency_ms, bool with_position = true)
  {
    const builtin_interfaces::msg::Time stamp = to_ros_time(est.stamp_ns);

    // Скорость (VelocitySensor, м/с).
    tram_vehicle_msgs::msg::VelocitySensor vel;
    vel.header.stamp = stamp;
    vel.header.frame_id = cfg_.child_frame_id;
    vel.velocity = est.velocity;
    pub_velocity_->publish(vel);

    // Положение (Odometry).
    if (with_position) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = cfg_.frame_id;
      odom.child_frame_id = cfg_.child_frame_id;
      odom.pose.pose.position.x = est.position.x;
      odom.pose.pose.position.y = est.position.y;
      odom.pose.pose.position.z = est.position.z;
      odom.pose.pose.orientation = heading_to_quaternion(est.heading);
      odom.pose.covariance = build_pose_covariance(est.position_cov);
      odom.twist.twist.linear.x = est.velocity;
      pub_position_->publish(odom);
    }

    // Отладочное состояние (Float64MultiArray с фиксированной раскладкой).
    if (cfg_.publish_debug) {
      std_msgs::msg::Float64MultiArray state;
      state.data = {
        est.velocity, est.acceleration,
        est.position.x, est.position.y, est.position.z,
        est.heading,
        static_cast<double>(est.slip),
        est.slip_confidence,
        static_cast<double>(est.quality),
        est.initialized ? 1.0 : 0.0,
        est.s,
        static_cast<double>(est.route_id),
        est.on_route ? 1.0 : 0.0,
        static_cast<double>(est.direction)
      };
      pub_state_->publish(state);

      std_msgs::msg::Float64 slip;
      slip.data = est.slip_confidence;
      pub_slip_->publish(slip);
    }

    // Диагностика.
    publish_diagnostics(est, latency_ms);
  }

  const Config & config() const {return cfg_;}

private:
  static builtin_interfaces::msg::Time to_ros_time(int64_t ns)
  {
    builtin_interfaces::msg::Time t;
    t.sec = static_cast<int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return t;
  }

  static geometry_msgs::msg::Quaternion heading_to_quaternion(double heading)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(heading * 0.5);
    q.w = std::cos(heading * 0.5);
    return q;
  }

  static std::array<double, 36> build_pose_covariance(double cov)
  {
    std::array<double, 36> c{};
    if (cov > 0.0) {
      c[0] = cov;   // x
      c[7] = cov;   // y
      c[14] = cov;  // z
    }
    return c;
  }

  void publish_diagnostics(const Estimate & est, double latency_ms)
  {
    diagnostic_msgs::msg::DiagnosticArray arr;
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "reserve_odometry";
    st.hardware_id = "tram";
    const bool ok = (est.quality == DataQuality::OK) && est.initialized;
    st.level = ok ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      (est.quality == DataQuality::STALE ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    st.message = ok ? "nominal" : std::string("quality=") + to_string(est.quality);

    add_kv(st, "data_quality", to_string(est.quality));
    add_kv(st, "slip", to_string(est.slip));
    add_kv(st, "slip_confidence", std::to_string(est.slip_confidence));
    add_kv(st, "latency_ms", std::to_string(latency_ms));
    add_kv(st, "initialized", est.initialized ? "true" : "false");
    add_kv(st, "velocity_m_s", std::to_string(est.velocity));
    arr.status.push_back(st);
    pub_diagnostics_->publish(arr);
  }

  static void add_kv(diagnostic_msgs::msg::DiagnosticStatus & st, const std::string & k, const std::string & v)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = k;
    kv.value = v;
    st.values.push_back(kv);
  }

  rclcpp::Node * node_;
  Config cfg_;
  rclcpp::Publisher<tram_vehicle_msgs::msg::VelocitySensor>::SharedPtr pub_velocity_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_position_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_state_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_slip_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__IO__OUTPUT_BRIDGE_HPP_

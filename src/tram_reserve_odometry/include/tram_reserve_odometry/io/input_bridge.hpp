// Входной мост: подписки на тележки и ручку -> потокобезопасный RawInputBuffer.
//   передняя/задняя тележка : tram_vehicle_msgs/msg/VelocitySensor (velocity, км/ч)
//   команда машиниста       : tram_vehicle_msgs/msg/DriverControllerCommand (position, int8),
//                             GenericSubscription + разбор CDR (io/driver_command.hpp)
// Образец хранит свой header.stamp (время bag), время приёма (steady clock) и сырое значение.

#ifndef TRAM_RESERVE_ODOMETRY__IO__INPUT_BRIDGE_HPP_
#define TRAM_RESERVE_ODOMETRY__IO__INPUT_BRIDGE_HPP_

#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tram_vehicle_msgs/msg/velocity_sensor.hpp>

#include "tram_reserve_odometry/core/types.hpp"
#include "tram_reserve_odometry/io/driver_command.hpp"
#include "tram_reserve_odometry/preprocessing/input_buffer.hpp"

namespace tram
{

class InputBridge
{
public:
  InputBridge(
    rclcpp::Node * node,
    const std::string & front_topic,
    const std::string & rear_topic,
    const std::string & driver_topic,
    int qos_depth = 10)
  : node_(node)
  {
    const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable();

    sub_front_ = node_->create_subscription<tram_vehicle_msgs::msg::VelocitySensor>(
      front_topic, qos,
      [this](const tram_vehicle_msgs::msg::VelocitySensor::SharedPtr msg) {
        on_velocity(msg, Channel::FRONT);
      });
    sub_rear_ = node_->create_subscription<tram_vehicle_msgs::msg::VelocitySensor>(
      rear_topic, qos,
      [this](const tram_vehicle_msgs::msg::VelocitySensor::SharedPtr msg) {
        on_velocity(msg, Channel::REAR);
      });
    try {
      sub_driver_ = node_->create_generic_subscription(
        driver_topic, kDriverCommandType, qos,
        [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {on_driver(*msg);});
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        node_->get_logger(), "No driver command subscription (%s is not available: %s); "
        "running without the controller handle.", kDriverCommandType, e.what());
    }
  }

  RawInputBuffer & buffer() {return buffer_;}

  /// Подать сообщение в обход подписки (офлайн-прогон).
  /// \param recv_ns время приёма (в офлайне — время записи в bag).
  void inject_velocity(
    const tram_vehicle_msgs::msg::VelocitySensor & msg, Channel c, int64_t recv_ns)
  {
    store_velocity(msg, c, recv_ns);
  }

  void inject_driver(const DriverCommand & msg, int64_t recv_ns)
  {
    store_driver(msg, recv_ns);
  }

private:
  void on_velocity(const tram_vehicle_msgs::msg::VelocitySensor::SharedPtr msg, Channel c)
  {
    store_velocity(*msg, c, steady_now_ns());
  }

  void on_driver(const rclcpp::SerializedMessage & raw)
  {
    const auto & m = raw.get_rcl_serialized_message();
    DriverCommand msg;
    if (decode_driver_command(m.buffer, m.buffer_length, msg)) {
      store_driver(msg, steady_now_ns());
    }
  }

  void store_velocity(
    const tram_vehicle_msgs::msg::VelocitySensor & msg, Channel c, int64_t recv_ns)
  {
    InputSample s;
    s.stamp_ns = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
      static_cast<int64_t>(msg.header.stamp.nanosec);
    s.value = msg.velocity;           // сырое значение, км/ч
    s.recv_ns = recv_ns;
    s.valid = true;
    s.quality = DataQuality::OK;
    buffer_.update(c, s);
  }

  void store_driver(const DriverCommand & msg, int64_t recv_ns)
  {
    InputSample s;
    s.stamp_ns = msg.stamp_ns();
    s.value = static_cast<double>(msg.position);  // сырое int8-деление в [-15, 15]
    s.recv_ns = recv_ns;
    s.valid = true;
    s.quality = DataQuality::OK;
    buffer_.update(Channel::DRIVER, s);
  }

  static int64_t steady_now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  rclcpp::Node * node_;
  RawInputBuffer buffer_;
  rclcpp::Subscription<tram_vehicle_msgs::msg::VelocitySensor>::SharedPtr sub_front_;
  rclcpp::Subscription<tram_vehicle_msgs::msg::VelocitySensor>::SharedPtr sub_rear_;
  rclcpp::GenericSubscription::SharedPtr sub_driver_;   ///< nullptr — ручки нет
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__IO__INPUT_BRIDGE_HPP_

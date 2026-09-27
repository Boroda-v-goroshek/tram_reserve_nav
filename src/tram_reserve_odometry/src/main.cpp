#include <rclcpp/rclcpp.hpp>

#include "tram_reserve_odometry/node/reserve_odometry_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tram::ReserveOdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"

#include "g1_locomotion/Inference.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<g1_locomotion::Inference>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

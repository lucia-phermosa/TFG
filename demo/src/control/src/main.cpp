#include <rclcpp/rclcpp.hpp>
#include "JointPubNode.hpp"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointPubNode>());
  rclcpp::shutdown();
  return 0;
}
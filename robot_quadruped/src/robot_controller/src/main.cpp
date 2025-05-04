#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuRepublisher : public rclcpp::Node
{
public:
  ImuRepublisher()
  : Node("robot_controller")
  {
    // Subscribe to the raw Gazebo IMU topic
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/world/robot_world/model/robot/link/base_link/sensor/imu_sensor/imu",
      10,
      std::bind(&ImuRepublisher::imu_callback, this, std::placeholders::_1));

    // Republish it to a standard /imu topic
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    RCLCPP_INFO(this->get_logger(), "IMU Republisher node started.");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_pub_->publish(*msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuRepublisher>());
  rclcpp::shutdown();
  return 0;
}


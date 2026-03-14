#pragma once

#include <rclcpp/rclcpp.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>

#include <chrono>
#include <vector>
#include <string>
#include <array>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <sstream>
#include <fstream>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <netdb.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include "Body.hpp"
#include "Leg.hpp"
#include "Gaits.hpp"
#include "Horizontal_controller.hpp"
#include "keyboard_cmd_parser.hpp"

using FJT = control_msgs::action::FollowJointTrajectory;

class JointPubNode : public rclcpp::Node {
public:
  JointPubNode();
  ~JointPubNode() override;

private:
  // --- TF helpers ---
  static inline Vec3 tfTransToVec3(const geometry_msgs::msg::TransformStamped& T);

  bool readLegOffsetsFromTF(tf2_ros::Buffer& tf_buffer,
                            const std::string& hombro,
                            const std::string& brazo,
                            const std::string& antebrazo,
                            const std::string& foot,
                            LegOffsets *out,
                            rclcpp::Logger logger,
                            rclcpp::Clock::SharedPtr clock);

  bool initLegOffsetsFromTF(tf2_ros::Buffer& tf_buffer,
                            rclcpp::Logger logger,
                            rclcpp::Clock::SharedPtr clock);

  void initTF();

  bool readFootPos(const std::string& foot_frame, float& x, float& y, float& z);
  void updateFeetFromTF();

  // --- Socket / commands ---
  void processSocket();
  void tick();

  static bool sendAllNonBlocking(int fd, const char* data, size_t len, rclcpp::Logger logger);
  static inline void appendNumber(std::ostringstream& oss, double v);
  bool sendTelemetryFrames();
  void openSocketAndListen();

private:
  std::shared_ptr<Body> body_;
  WalkGait gait_;
  Horizontal_controller horiz_control;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::CreateTimerROS> tf_timer_;

  bool body_initialized_{false};
  bool offsets_initialized_{false};

  double start_delay_s_ = 10.0;
  rclcpp::Time start_time_;
  bool started_ = false;

  bool walk_ON{false};

  float goalX = 0.0f, goalY = 0.0f;
  float step_x{0.0f}, step_y{0.0f};
  float step{0.02f};

  bool trot_gait = false;
  rclcpp::Time trot_start;
  float trot_t_last_s = 0.0f;
  float last_time = 0;
  float yaw_corr = 0;
  float t_sim = 0;

  float osc_w     = 2.0f * static_cast<float>(M_PI) * 1.8f;
  float oscZ_amp  = 0.04f;
  float oscX_amp  = 0.03f;
  float oscY_amp  = 0.0f;
  float osc_phi   = static_cast<float>(M_PI);
  float oscX_phi  = 0.0f;
  float oscY_phi  = 0.0f;

  float dx{}, dy{}, dz{};

  bool  newPose = false;
  float roll{0.0f}, pitch{0.0f}, yaw{0.0f};
  float roll_meas{0.0f}, pitch_meas{0.0f}, yaw_meas{0.0f};
  float roll_corr{0.0f}, pitch_corr{0.0f};
  float robotX = 0.0f, robotY = 0.0f, robotZ = 0.0f;
  float body_x = 0.0f, body_y = 0.0f, body_z = 0.0f;
  float err_x = 0.0f, err_y = 0.0f, err_z = 0.0f;
  float height{-0.134f};

  float roll_cmd{0.0f};
  float pitch_cmd{0.0f};

  bool telem_ON_ = false;
  bool isMoving = false;
  bool centerBody = false;

  // IMU
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::string imu_topic_{"/imu"};
  std::atomic<double> imu_roll_{0.0}, imu_pitch_{0.0};

  // ODOM
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::string odom_topic_{"/odom"};
  std::atomic<double> odom_x_{0}, odom_y_{0}, odom_z_{0};
  std::atomic<double> odom_yaw_{0};

  // Model name
  std::string model_name_{};

  std::atomic<double> current_x_{0.0}, current_y_{0.0}, current_z_{0.0};

  std::vector<std::string> joint_names_;
  std::chrono::duration<double> cycle_dt_{0.02};
  double traj_lookahead_s_{0.05};

  int64_t socket_port_ = 9000;
  std::string controller_name_{"joint_trajectory_controller"};
  bool publish_js_{false};

  int listen_fd_ = -1;
  int client_fd_ = -1;
  int conn_fd_   = -1;
  std::unique_ptr<CommandParser> parser_;

  std::ofstream csv_file3_;
  bool logging_active3_ = true;
  std::ofstream csv_file2_;
  bool logging_active2_ = false;
  std::ofstream csv_file_;
  bool logging_active_ = false;
  bool csv_opened_ = false;

  double log_period_s_ = 0.20;
  double last_log_t_ = -1.0;

  int64_t now_ms{};
  int64_t last_telem_ms_ = 0;
  int64_t telem_rate_ms_ = 50;

  float moveBody = false;
};
#pragma once

#include <array>
#include <string>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

#include "Enums.hpp"
#include "Kinematics.hpp"
#include "Planner.hpp"

class Leg {
public:
  // --- Usado desde fuera (Body / JointPubNode) ---
  FootPlanner footPlanner{};

  bool walk{false};

  // IK / ángulos 
  float q0{0.0f};
  float q1{0.0f};
  float q2{0.0f};

  float ang1{0.0f};
  float ang2{0.0f};
  float ang3{0.0f};

  // Foot pose (home/meas/cmd) 
  float foot_home_x{0.0f};
  float foot_home_y{0.0f};
  float foot_home_z{0.0f};

  float foot_meas_x{0.0f};
  float foot_meas_y{0.0f};
  float foot_meas_z{0.0f};

  float foot_cmd_x{0.0f};
  float foot_cmd_y{0.0f};
  float foot_cmd_z{0.0f};

  // Offsets leídos desde TF (Body/JointPubNode los rellenan)
  std::array<LegOffsets, 4> leg_offsets_{};

  // Constructor
  Leg(LegID id,
      float length_l0, float length_l1, float length_l2,
      Side side, End end,
      const rclcpp::Logger& logger);
  ~Leg() = default;

  // Identidad
  LegID get_id()  const { return id_; }
  Side  get_side() const { return side_; }
  End   get_end()  const { return end_; }

  // ---- Joint names (orden: [elbow, shoulder, hip]) ----
  std::array<std::string, 3>&       joint_names()       { return joint_names_; }
  const std::array<std::string, 3>& joint_names() const { return joint_names_; }

  void set_joint_names(const std::array<std::string, 3>& names) { joint_names_ = names; }
  void set_joint_names(const std::string& elbow,
                       const std::string& shoulder,
                       const std::string& hip)
  {
    joint_names_ = {{ elbow, shoulder, hip }};
  }

  // ---- Joint positions (rad, orden: [elbow, shoulder, hip]) ----
  std::array<float, 3>&       joint_positions_rad()       { return joint_pos_rad_; }
  const std::array<float, 3>& joint_positions_rad() const { return joint_pos_rad_; }

  void set_joint_positions_rad(const std::array<float, 3>& rad);
  void set_joint_positions_deg(const std::array<float, 3>& deg);

  // API usada en Body/Gaits
  void update(float pX, float pY, float pZ);
  void go2pos(float px, float py, float pz, float dz, int type = absolute);
  bool arrived2pos();

  void setJointsAngle(float ang_sh1_deg,
                      float ang_sh2_deg,
                      float ang_elbow_deg,
                      bool limits = true);

  void setFootPos(float px, float py, float pz);

  // Utils
  static float deg2rad(float deg);
  static float clamp(float x, float lo, float hi);
  void clampAll();

private:
  // Identidad/orientación
  LegID id_{FR};
  Side  side_{Side::right};
  End   end_{End::front};

  // Geometría (no usada en este .cpp, pero la inicializas en el ctor)
  float l0{0.0f};
  float l1{0.0f};
  float l2{0.0f};

  // Joint state + límites
  std::array<std::string, 3> joint_names_{{"", "", ""}};
  std::array<float, 3> joint_pos_rad_{{0.0f, 0.0f, 0.0f}};

  std::array<float, 3> min_rad_{{-0.785f, -0.785f, -0.785f}};
  std::array<float, 3> max_rad_{{ 0.785f,  0.785f,  0.785f}};

  // Planner speed (usada en go2pos)
  float foot_speed{50.0f};

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("Leg")};
};
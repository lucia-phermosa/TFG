#pragma once

#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "Enums.hpp"
#include "Leg.hpp"
#include "Planner.hpp"

class Body {
public:
  // --- Datos usados desde fuera (JointPubNode / gait / control) ---
  std::array<Vec3, 4> hips{};

  float body_width{0.189f};
  float body_length{0.2f};

  float worldX{0.0f};
  float worldY{0.0f};
  float worldZ{0.0f};

  float posX{0.0f};
  float posY{0.0f};
  float posZ{0.0f};

  float roll{0.0f};
  float pitch{0.0f};
  float yaw{0.0f};

  float cg_x{0.0f};
  float cg_y{0.0f};

  // Estado usado por tu lógica
  int  active_leg_{-1};
  int  movementState{0};
  bool isIdle{true};

  // Planner (se usa en Body.cpp)
  TrayectoryPlanner balancePlanner;

  // Patas (se accede desde fuera: body_->legs_[FR]...)
  std::array<std::shared_ptr<Leg>, 4> legs_{};
  std::array<std::shared_ptr<Leg>, 4>& legs() { return legs_; }
  const std::array<std::shared_ptr<Leg>, 4>& legs() const { return legs_; }

public:
  explicit Body(const rclcpp::Node::SharedPtr& node);
  ~Body() = default;

  void set_body_size(float length_mm, float width_mm);
  void setup();

  // Espera 12 nombres en orden: FR(0..2), RR(3..5), RL(6..8), FL(9..11)
  void assign_joint_names_from_array(const std::vector<std::string>& names);

  // Orden de 'deg': FL(0..2), RL(3..5), RR(6..8), FR(9..11)
  void set_initial_joint_angles_deg(std::array<float, 12>& deg);

  void move(float dx, float dy, float dz);
  void go2pos(float px, float py, float pz, int type = absolute);
  bool arrived2pos();
  void updatePose(bool apply_body_rpy);
  void getMeanFootPos(float* x, float* y, float* z);
  bool centerPos();
  bool moveFoot(int legID,float px, float py, float pz,float dz_lift_in, int movementType);

  Leg* leg_at(std::size_t i);

private:
  // --- ROS infra ---
  rclcpp::Node::WeakPtr node_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  // --- Constantes / tunings usados en Body.cpp ---
  const float body_speed{50.0f}; // (tu comentario: mm/ms)

  // --- Estado interno usado por moveFoot() (aunque no lo declares aquí, tu .cpp lo usa) ---
  int   stable_count_{0};
  bool  body_cmd_sent_{false};
  float body_target_x_{0.0f};
  float body_target_y_{0.0f};
  float swing_start_x_{0.0f};
  float swing_start_y_{0.0f};

};
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <array>
#include <string>
#include "Leg.hpp"
#include "Planner.hpp"
#include "Enums.hpp"

class Body {
public:
  // float body_width = 90.0f;
  // float body_length = 184.0f;
  float body_width = 2756.0f;
  float body_length = 5635.0f;

  float posX = 0, posY = 0, posZ = 0; //in millimeters
  float roll = 0, pitch = 0, yaw = 0;
  float camber = 0.0;

  const float body_speed = 0.08f; // mm/ms
  int movemetState = 0;
  int centeringState = 0;
  bool isIdle = true;

  Body(const rclcpp::Node::SharedPtr& node);
  ~Body() = default;

  void set_body_size(float length_mm, float width_mm);
  void setup();

  void set_neutral_stance(int x_front, int x_rear, int y_offset, int z_down);
  // Espera 12 nombres en orden: FL[H,B,A], RL[H,B,A], RR[H,B,A], FR[H,B,A]
  void assign_joint_names_from_array(const std::vector<std::string>& names);

  // Espera 12 ángulos en grados en orden: FL[H,B,A], RL[H,B,A], RR[H,B,A], FR[H,B,A]
  void set_initial_joint_angles_deg(std::array<double,12>& deg);

  // Hook para cinemática/sensores; aquí puede ser no-op
  void setCamber(float camber_in_mm);
  void move(float dx, float dy, float dz);
  void go2pos(float px, float py, float pz, int type = absolute);
  bool arrived2pos();
  void updatePose();
  void getMeanFootPos(float *x, float *y, float *z);
  bool moveFoot(int legID, float px, float py, float pz, int dz, int movementType = absolute, bool return2zero = true);
  int getAbsFootPos(int legID);
  bool centerPos(float dx=0, float dy=0, float dz=0);

  // Acceso a patas
  std::array<std::shared_ptr<Leg>, 4> legs_{};  // indexadas por LegID
  std::array<std::shared_ptr<Leg>, 4>& legs() { return legs_; }
  const std::array<std::shared_ptr<Leg>, 4>& legs() const { return legs_; }
  
  // Utilidad
  static double deg2rad(double d) { return d * 3.14159265358979323846 / 180.0; }

  Leg* leg_at(std::size_t i);
  const Leg* leg_at(std::size_t i) const;

private:
  rclcpp::Node::WeakPtr node_;
  rclcpp::Logger logger_;
  TrayectoryPlanner balancePlanner;
  int centerPosX = 0;
  int centerPosY = 0;
  bool resetFootState = 0;
};

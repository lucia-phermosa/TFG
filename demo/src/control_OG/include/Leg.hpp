#pragma once
#include <array>
#include <string>
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "Kinematics.hpp"
#include "Planner.hpp"
#include "Enums.hpp"

class Leg {
public:
  // Estado público existente
  float next_pX = 0, next_pY = 0, next_pZ;
  int wlk_state = 1;

  float footX = 0, footY = 0, footZ = 0;
  int q0_offset = 0, q1_offset = 0, q2_offset = 0;

  // Constructor: cada pata conoce su LegID, Side y End
  Leg(LegID id, int length_l0, int length_l1, int length_l2,
      Side side, End end, const rclcpp::Logger& logger);
  ~Leg() = default;

  // Identidad y orientación
  LegID get_id()  const { return id_; }
  Side  get_side() const { return side_; }
  End   get_end()  const { return end_; }

  // ---- Joint names (orden interno SIEMPRE: [sh1, sh2, elbow]) ----
  std::array<std::string,3>&       joint_names()       { return joint_names_; }
  const std::array<std::string,3>& joint_names() const { return joint_names_; }

  void set_joint_names(const std::array<std::string,3>& names) { joint_names_ = names; }
  void set_joint_names(const std::string& hombro,
                       const std::string& brazo,
                       const std::string& antebrazo) {
    joint_names_ = {{hombro, brazo, antebrazo}}; // [sh1, sh2, elbow]
  }
  const std::string& joint_name(Joint j) const { return joint_names_[static_cast<size_t>(j)]; }

  // ---- Joint positions (radianes, orden interno: [sh1, sh2, elbow]) ----
  std::array<double,3>&       joint_positions_rad()       { return joint_pos_rad_; }
  const std::array<double,3>& joint_positions_rad() const { return joint_pos_rad_; }

  void set_joint_positions_rad(const std::array<double,3>& rad);
  void set_joint_positions_deg(const std::array<double,3>& deg);

  // Útil si te llegan como [elbow, sh2, sh1] (p.ej. de Revolution N)
  void set_joint_positions_deg_from_elbow_sh2_sh1(const std::array<double,3>& esh2sh1);

  // Acceso por índice semántico
  double&       pos(Joint j)       { return joint_pos_rad_[static_cast<size_t>(j)]; }
  const double& pos(Joint j) const { return joint_pos_rad_[static_cast<size_t>(j)]; }

  // ---- Resto de API existente ----
  void initAbsFootPos(float bodyLength, float bodyWidth);
  void update(float pX, float pY, float pZ);
  void go2pos(float px, float py, float pz, int dz, int type = absolute);
  bool arrived2pos();
  void resetWalk();
  void setJointsAngle(double ang_sh1_deg, double ang_sh2_deg, double ang_elbow_deg, bool limits = true);
  void setFootPos(float px, float py, float pz);
  void getFoot2ReffVect(float bodyX, float bodyY, float *distX, float *distY);

  static double deg2rad(double deg);
  static double rad2deg(double rad);
  static double clamp(double x, double lo, double hi);
  void clampAll();

private:
  // Identidad/orientación
  LegID id_;
  Side  side_;
  End   end_;

  // Geometría y estado interno
  int l0{}, l1{}, l2{};
  int q0{}, q1{}, q2{};
 
  float foot_speed = 0.4f; // mm/ms
  FootPlanner footPlanner;

  // Posición relativa del hombro respecto al centro del cuerpo
  float shoulderPosX{};
  float shoulderPosY{};

  // Tiempos
  unsigned long now{};
  unsigned long last = 0;
  unsigned long dt   = 5;  // ms entre puntos de trayectoria (min 5)

  // Logger y datos de joints
  rclcpp::Logger logger_{rclcpp::get_logger("Leg")};

  // Orden interno de joints: [sh1, sh2, elbow]
  std::array<std::string,3> joint_names_{{"", "", ""}};
  std::array<double,3>      joint_pos_rad_{{0.0, 0.0, 0.0}};
  std::array<double,3>      min_rad_{{ -1.57, -1.57, -1.57 }};
  std::array<double,3>      max_rad_{{  1.57,  1.57,  1.57 }};
};

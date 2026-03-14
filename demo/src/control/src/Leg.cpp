#include "Leg.hpp"
#include <cmath>

// ---- Utilidades ----
float Leg::deg2rad(float deg) { return static_cast<float>(deg * M_PI / 180.0); }
float Leg::clamp(float x, float lo, float hi) {
  return std::max(lo, std::min(x, hi));
}

Leg::Leg(LegID id, float length_l0, float length_l1, float length_l2,
         Side side, End end, const rclcpp::Logger& logger)
: id_(id),
  side_(side),
  end_(end),
  l0(length_l0),
  l1(length_l1),
  l2(length_l2),
  logger_(logger)
{
  RCLCPP_INFO(logger_, "Leg creada: id=%d, l0=%f, l1=%f, l2=%f, side=%d, end=%d",
              static_cast<int>(id_), l0, l1, l2,
              static_cast<int>(side_), static_cast<int>(end_));
}

// ================= Estado articular ==============
void Leg::set_joint_positions_rad(const std::array<float,3>& rad) {
  joint_pos_rad_ = rad;        // Se asume orden [elbow, sh, hip]
  // clampAll();               // <-- descomenta si quieres aplicar límites aquí
}

void Leg::set_joint_positions_deg(const std::array<float,3>& deg) {
  std::array<float,3> r{{ deg2rad(deg[0]), deg2rad(deg[1]), deg2rad(deg[2]) }};
  set_joint_positions_rad(r);
}

void Leg::clampAll() {
  for (int i = 0; i < 3; ++i) {
    joint_pos_rad_[i] = clamp(joint_pos_rad_[i], min_rad_[i], max_rad_[i]);
  }
}

void Leg::update(float pX, float pY, float pZ)
{
  const auto& off = leg_offsets_[static_cast<int>(id_)];
  // Añadir signo para q0??
  getJoint_byPos(logger_, pX, pY, pZ, off, &q0, &q1, &q2);

  if(walk)
  {
    q0 = 1.1f*q0;
    q1 = 1.1f*q1;
    q2 = 2.0f*q2;
  }

  setJointsAngle(q0, q1, q2);
}

// // ================= Planner simple =================
void Leg::go2pos(float px, float py, float pz, float dz, int type){
    if (type == absolute){
        footPlanner.setPlanner(logger_, foot_speed, foot_cmd_x, foot_cmd_y, foot_cmd_z, px, py, pz, dz);
    }
    else if (type == incremental){
        footPlanner.setPlanner(logger_, foot_speed, foot_cmd_x, foot_cmd_y, foot_cmd_z, foot_cmd_x+px, foot_cmd_y+py, foot_cmd_z+pz, dz);
    }
}

bool Leg::arrived2pos(){
  bool isDone = footPlanner.updatePos(logger_);
  foot_cmd_x = footPlanner.newX;
  foot_cmd_y = footPlanner.newY;
  foot_cmd_z = footPlanner.newZ;
  
  return isDone;
}

void Leg::setJointsAngle(float ang_sh1_deg, float ang_sh2_deg, float ang_elbow_deg, bool limits) {
  // Mapping original por lado (en GRADOS), luego convertimos a rad

  ang1 = ang_sh1_deg;
  ang2 = ang_sh2_deg;
  ang3 = ang_elbow_deg;

  if (limits) {
    ang1 = clamp(ang1, min_rad_[0], max_rad_[0]);
    ang2 = clamp(ang2, min_rad_[1], max_rad_[1]);
    ang3 = clamp(ang3, min_rad_[2], max_rad_[2]);
  }

  // RCLCPP_INFO(logger_, "AFTER LIMITS -> hip: %.3f, shoulder: %.3f, elbow: %.3f\n", ang1, ang2, ang3);

  joint_pos_rad_[Joint::hip]   = ang1;
  joint_pos_rad_[Joint::sh]   = ang2;
  joint_pos_rad_[Joint::elbow] = ang3;
}

void Leg::setFootPos(float px, float py, float pz){
    foot_cmd_x = px;
    foot_cmd_y = py;
    foot_cmd_z = pz;
}

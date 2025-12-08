#include "Leg.hpp"


// ---- Utilidades ----
double Leg::deg2rad(double deg) { return deg * M_PI / 180.0; }
double Leg::rad2deg(double rad) { return rad * 180.0 / M_PI; }
double Leg::clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(x, hi));
}

// ================= Constructores =================
// Leg::Leg(const rclcpp::Node::SharedPtr& node, std::string& id)
// : logger_(node->get_logger()), id_(id)
// {
//   joint_pos_rad_[0] = ang1; 
//   joint_pos_rad_[1] = ang2; 
//   joint_pos_rad_[2] = ang3;
// }

Leg::Leg(LegID id, int length_l0, int length_l1, int length_l2,
         Side side, End end, const rclcpp::Logger& logger)
: id_(id),
  side_(side),
  end_(end),
  l0(length_l0),
  l1(length_l1),
  l2(length_l2),
  logger_(logger)
{
  RCLCPP_INFO(logger_, "Leg creada: id=%d, l0=%d, l1=%d, l2=%d, side=%d, end=%d",
              static_cast<int>(id_), l0, l1, l2,
              static_cast<int>(side_), static_cast<int>(end_));
}

// ================= Estado articular ==============
void Leg::set_joint_positions_rad(const std::array<double,3>& rad) {
  joint_pos_rad_ = rad;        // Se asume orden [sh1, sh2, elbow]
  // clampAll();               // <-- descomenta si quieres aplicar límites aquí
}

void Leg::set_joint_positions_deg(const std::array<double,3>& deg) {
  std::array<double,3> r{{ deg2rad(deg[0]), deg2rad(deg[1]), deg2rad(deg[2]) }};
  set_joint_positions_rad(r);
}

// Opcional: para entrada en orden [elbow, sh2, sh1]
void Leg::set_joint_positions_deg_from_elbow_sh2_sh1(const std::array<double,3>& esh2sh1) {
  // Reordenar a [sh1, sh2, elbow] y convertir a rad
  std::array<double,3> hba_deg{{ esh2sh1[2], esh2sh1[1], esh2sh1[0] }};
  set_joint_positions_deg(hba_deg);
}

void Leg::clampAll() {
  for (int i = 0; i < 3; ++i) {
    joint_pos_rad_[i] = clamp(joint_pos_rad_[i], min_rad_[i], max_rad_[i]);
  }
}

// ================= Geometría base/pie ============
void Leg::initAbsFootPos(float bodyLength, float bodyWidth){
  const float end_sign  = static_cast<float>(static_cast<int>(end_));
  const float side_sign = static_cast<float>(static_cast<int>(side_));

  // l0 es int -> cast explícito a float
  const float l0f = static_cast<float>(l0);

  // Evitar división entera y conversiones implícitas
  shoulderPosX = -0.5f * bodyLength * end_sign;
  shoulderPosY = (0.5f * bodyWidth + l0f) * side_sign;

  footX = shoulderPosX;
  footY = shoulderPosY;
  footZ = 0.0f;
}

void Leg::update(float pX, float pY, float pZ){
    // RCLCPP_INFO(logger_, "Side: %d\n", side);
    if (side_ == right){
      getJoint_byPos(pX, pY, pZ, l0, l1, l2, &q0, &q1, &q2);
      setJointsAngle(q0, 180 + q1, q2);
      // RCLCPP_INFO(logger_, "Righ Joints q0 = %f, q1 = %f, q2 = %f\n", joint_pos_rad_[0], joint_pos_rad_[1], joint_pos_rad_[2]);
    }
    else if (side_ == left){
      getLeftJoint_byPos(pX, pY, pZ, l0, l1, l2, &q0, &q1, &q2);
      setJointsAngle(q0, 180 - q1, -q2);
      // RCLCPP_INFO(logger_, "Left Joints q0 = %f, q1 = %f, q2 = %f\n", joint_pos_rad_[0], joint_pos_rad_[1], joint_pos_rad_[2]);
    }
}

// // ================= Planner simple =================
void Leg::go2pos(float px, float py, float pz, int dz, int type){
    if (type == absolute){
        footPlanner.setPlanner(foot_speed, footX, footY, footZ, px, py, pz, dz);
    }
    else if (type == incremental){
        footPlanner.setPlanner(foot_speed, footX, footY, footZ, footX+px, footY+py, footZ+pz, dz);
    }
}

bool Leg::arrived2pos(){
    int isDone = footPlanner.updatePos();
    footX = footPlanner.newX;
    footY = footPlanner.newY;
    footZ = footPlanner.newZ;
    if (isDone){
        return true;
    }
    return false;
}

void Leg::resetWalk(){
    next_pX = 0;
    next_pY = 0;
}

void Leg::setJointsAngle(double ang_sh1_deg, double ang_sh2_deg, double ang_elbow_deg, bool limits) {
  // Mapping original por lado (en GRADOS), luego convertimos a rad
  double ang1, ang2, ang3;

  if (side_ == right) {
    ang1 = 90.0 + 2.0 * ang_sh1_deg * static_cast<int>(end_);
    ang2 = 160.0 - 2.0 * ang_sh2_deg;
    ang3 = -80.0 + 2.0 * ang_elbow_deg;
  } else { // left
    ang1 = 90.0 + 2.0 * ang_sh1_deg * static_cast<int>(end_);
    ang2 = 20.0 + 2.0 * ang_sh2_deg;
    ang3 = 260.0 - 2.0 * ang_elbow_deg;
  }

  if (limits) {
    double min_deg[3] = { rad2deg(min_rad_[0]), rad2deg(min_rad_[1]), rad2deg(min_rad_[2]) };
    double max_deg[3] = { rad2deg(max_rad_[0]), rad2deg(max_rad_[1]), rad2deg(max_rad_[2]) };
    ang1 = clamp(ang1, min_deg[0], max_deg[0]);
    ang2 = clamp(ang2, min_deg[1], max_deg[1]);
    ang3 = clamp(ang3, min_deg[2], max_deg[2]);
  }

  joint_pos_rad_[Joint::sh1]   = deg2rad(ang1);
  joint_pos_rad_[Joint::sh2]   = deg2rad(ang2);
  joint_pos_rad_[Joint::elbow] = deg2rad(ang3);
}

void Leg::setFootPos(float px, float py, float pz){
    footX = px;
    footY = py;
    footZ = pz;
}

void Leg::getFoot2ReffVect(float bodyX, float bodyY, float *distX, float *distY){
    *distX = bodyX + shoulderPosX - footX;
    *distY = bodyY + shoulderPosY - footY;
}

// void Leg::setServoAngle(int joint_index, double angle_deg) {
//   if (joint_index < 0 || joint_index > 2) return;
//   setJointRad(static_cast<Joints>(joint_index), deg2rad(angle_deg));
// }

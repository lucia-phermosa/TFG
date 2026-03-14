#include "Body.hpp"

Body::Body(const rclcpp::Node::SharedPtr& node)
: node_(node), logger_(node->get_logger()), clock_(node->get_clock()), balancePlanner(node->get_clock())
{
}

static inline float clampf(float v, float a, float b) { return std::max(a, std::min(b, v)); }

void Body::set_body_size(float length_mm, float width_mm) {
  body_length = length_mm;
  body_width  = width_mm;
}

void Body::set_initial_joint_angles_deg(std::array<float,12>& d){
  // Orden de 'd': FL(0..2), RL(3..5), RR(6..8), FR(9..11) 

  std::array<float,3> qFR{{d[0], d[1], d[2]}};
  std::array<float,3> qRR{{d[3], d[4], d[5]}};
  std::array<float,3> qRL{{d[6], d[7], d[8]}};
  std::array<float,3> qFL{{d[9], d[10], d[11]}};

  legs_[FL]->set_joint_positions_deg(qFL);
  legs_[RL]->set_joint_positions_deg(qRL);
  legs_[RR]->set_joint_positions_deg(qRR);
  legs_[FR]->set_joint_positions_deg(qFR);
}

void Body::setup() {
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("Body"), "Node expirado en Body::setup()");
    return;
  }

  // FR=0, RR=1, RL=2, FL=3 (orden de tu enum)
  constexpr std::array<std::pair<Side,End>, 4> kLegPose{{
    { Side::right, End::front }, // FR
    { Side::right, End::rear  }, // RR
    { Side::left,  End::rear  }, // RL
    { Side::left,  End::front }  // FL
  }};

  for (int i = 0; i < 4; ++i) {
    auto id = static_cast<LegID>(i);
    auto [side, end] = kLegPose[i];
    if (!legs_[i]) {
    //   legs_[i] = std::make_shared<Leg>(id, 40, 50, 50, side, end, node->get_logger());
        legs_[i] = std::make_shared<Leg>(id, 0.02, 0.1, 0.1, side, end, node->get_logger());
    }

    // Asignar nombres de joint consistentes con el orden [sh1, sh2, elbow]
    // Ej: "FR_sh1", "FR_sh2", "FR_elbow"
    const std::array<const char*, 4> leg_id_str{{"FR","RR","RL","FL"}};

    std::array<std::string, 3> jnames{{
        std::string(leg_id_str[i]) + "_elbow",
        std::string(leg_id_str[i]) + "_shoulder",
        std::string(leg_id_str[i]) + "_hip"
    }};
    legs_[i]->set_joint_names(jnames);

  }

  RCLCPP_INFO(logger_, "Body setup complete.");
}

void Body::move(float dx, float dy, float dz)
{
  posX += dx;
  posY += dy;
  posZ += dz;

  if (std::fabs(dx) > 1e-9f || std::fabs(dy) > 1e-9f || std::fabs(dz) > 1e-9f) {
    for (int i = 0; i < 4; ++i) {
      if (i == !active_leg_) continue;      // no tocar pata en swing
      auto &leg = legs_[i];
      if (!leg) continue;

      leg->foot_cmd_x += dx;
      leg->foot_cmd_y += dy;
      leg->foot_cmd_z += dz;
    }
  }
}

void Body::go2pos(float px, float py, float pz, int type)
{
  if (type == absolute) {
    balancePlanner.setPlanner(logger_, body_speed, posX, posY, posZ, px, py, pz);
  } else { // incremental
    balancePlanner.setPlanner(logger_, body_speed, posX, posY, posZ, posX + px, posY + py, posZ + pz);
  }
  isIdle = false;
}

bool Body::arrived2pos()
{
  const float eps_xy = 0.005f;
  const float eps_z = 0.005f;
  
  const float ex = balancePlanner.targetX - posX;
  const float ey = balancePlanner.targetY - posY;
  const float ez = balancePlanner.targetZ - posZ;

  const float e_xy = std::sqrt(ex*ex + ey*ey);
  const float e_z  = std::abs(ez);

  const bool planner_done = !balancePlanner.isActive();
  return planner_done && (e_xy < eps_xy) && (e_z < eps_z);
}

void Body::updatePose(bool apply_body_rpy)
{
  balancePlanner.computeNewPos();
  posX = balancePlanner.newX;
  posY = balancePlanner.newY;
  posZ = 0.0f;;

  auto do_leg = [&](int i)
  {
    // 1) Pie objetivo en base_link
    float fx = legs_[i]->foot_cmd_x;
    float fy = legs_[i]->foot_cmd_y;
    float fz = legs_[i]->foot_cmd_z;

    // 2) Vector relativo hip->foot (en base_link)
    const auto& hip = hips[i];
    float pX, pY, pZ;
    // float pX = fx - hip.x;
    // float pY = fy - hip.y;
    // float pZ = fz - hip.z;

    if (apply_body_rpy) {
      // computeYaw(yaw, &pX, &pY);

      computePitch(pitch, &fx, &fz);

      computeRoll(roll, &fy, &fz);
      
    }
    pX = fx - hip.x;
    pY = fy - hip.y;
    pZ = fz - hip.z;

    // 3) IK en frame hip (pX,pY,pZ)
    legs_[i]->update(pX, pY, pZ);
  };


  do_leg(FR);
  do_leg(RR);
  do_leg(RL);
  do_leg(FL);
}

void Body::getMeanFootPos(float* x, float* y, float* z) 
{
  float sx=0.f, sy=0.f, sz=0.f;
  for (int i=0;i<4;++i) {
    sx += legs_[i]->foot_home_x;
    sy += legs_[i]->foot_home_y;
    sz += legs_[i]->foot_home_z;
  }
  *x = sx * 0.25f;
  *y = sy * 0.25f;
  *z = sz * 0.25f;
}

bool Body::centerPos(){
  for(int i = 0; i < 4; i++)
  {
    legs_[i]->foot_cmd_x = legs_[i]->foot_home_x;
    legs_[i]->foot_cmd_y = legs_[i]->foot_home_y;
    legs_[i]->foot_cmd_z = legs_[i]->foot_home_z;
  }

  return true;
}

void Body::assign_joint_names_from_array(const std::vector<std::string>& names){
  if (names.size() != 12){
    RCLCPP_WARN(logger_, "Esperaba 12 joint names, recibidos: %zu", names.size());
    return;
  }

  auto set_block = [&](LegID id, size_t base){
    if (!legs_[id]) {
      RCLCPP_WARN(logger_, "Leg %d no inicializada al asignar names", static_cast<int>(id));
      return;
    }
    // Entrada por bloque: [elbow, sh2, sh1] -> interno: [sh1, sh2, elbow]
    const std::string& elbow = names[base + 0];
    const std::string& sh    = names[base + 1];
    const std::string& hip   = names[base + 2];
    // RCLCPP_INFO(logger_, "ID: %d -> Elbow: %s, Shoulder: %s, Hip: %s\n", id, elbow.c_str(), sh1.c_str(), sh2.c_str());
    legs_[id]->set_joint_names(elbow, sh, hip);
  };

  // Orden de entrada de patas: FR (0..2), RR (3..5), RL (6..8), FL (9..11)
  set_block(FR, 0);
  set_block(RR, 3);
  set_block(RL, 6);
  set_block(FL, 9);
}

Leg* Body::leg_at(std::size_t i){
  if (i < legs_.size() && legs_[i]) return legs_[i].get();
  RCLCPP_WARN(logger_, "leg_at(%zu) inválida o nula", i);
  return nullptr;
}

bool Body::moveFoot(int legID,
                    float px, float py, float pz,
                    float dz_lift_in,
                    int movementType)
{
  // ------------------ Tuning ------------------
  const float min_lift = 0.03f;
  const float max_lift = 0.1f;

  // Touchdown Z (solo como “corrección opcional”, sin bloquear)
  // const float lift_zfix_touch = 0.010f; // lift usado para corrección de Z

  // Body move (con CG)
  // const float body_alpha_step = 1.0f;
  float alpha        = 0.60f;
  const float body_max_inc    = 0.1f;
  const int   body_stable_cycles = 1;

  // convención tuya para el swing
  px = -px;

  // Exclusión: una pierna activa
  if (movementState != 0) {
    if (active_leg_ != legID) return false;
  } else {  
    active_leg_ = legID;
  }

  // ---------------- STATE 0: lanzar swing (planner de la pata) ----------------
  if (movementState == 0)
  {
    // touchdown_fix_sent_ = false;
    body_cmd_sent_      = false;
    stable_count_       = 0;

    swing_start_x_ = legs_[legID]->foot_meas_x;
    swing_start_y_ = legs_[legID]->foot_meas_y;

    float dz_lift = dz_lift_in;
    const float z_ref  = legs_[legID]->foot_home_z;
    const float stride = std::sqrt(px*px + py*py);
    dz_lift = std::max(dz_lift, 0.25f * std::fabs(z_ref));
    dz_lift = std::max(dz_lift, 0.50f * stride);
    dz_lift = std::clamp(dz_lift, min_lift, max_lift);

    // px = px + 0.03f;
    legs_[legID]->go2pos(px, py, pz, dz_lift, movementType);

    // RCLCPP_INFO(logger_, "[moveFoot:S0] leg=%d swing cmd (%.3f,%.3f,%.3f) lift=%.3f",
    //             legID, px, py, pz, dz_lift);

    movementState = 1;
    return false;
  }

  // ---------------- STATE 1: esperar fin del swing (sin bloquear por home_z) ----------------
  if (movementState == 1)
  {
    if (!legs_[legID]->arrived2pos())
      return false;

    // 1) Centroide del triángulo de apoyo (promedio de 3 patas)
    float supCx = 0.f, supCy = 0.f;
    int n = 0;
    for (int i = 0; i < 4; ++i) {
      if (i == legID) continue;
      supCx += legs_[i]->foot_meas_x;
      supCy += legs_[i]->foot_meas_y;
      ++n;
    }
    if (n != 3) return false;
    supCx *= (1.f / 3.f);
    supCy *= (1.f / 3.f);

    // 2) Delta de cuerpo requerido.
    //    Mover el cuerpo en +dx hace que el CG proyectado (respecto al suelo)
    //    se desplace en -dx en base_link, por eso:
    //      cg' = cg - delta_body  y queremos cg' = supC  => delta_body = cg - supC
    float dx_body = (supCx + cg_x);
    float dy_body = (supCy + cg_y);
    
    
    // 3) Suavizado y clamp por incremento
    alpha = clampf(alpha, 0.f, 1.f);
    dx_body *= alpha;
    dy_body *= alpha;
    
    dx_body = clampf(dx_body, -body_max_inc, body_max_inc);
    dy_body = clampf(dy_body, -body_max_inc, body_max_inc);
    
    // 4) Target absoluto del cuerpo (mismo frame que posX/posY)
    body_target_x_ = posX + dx_body + 0.05f;
    body_target_y_ = posY + dy_body;
    // body_target_x_ = dx_body;
    // body_target_y_ = dy_body;

    go2pos(body_target_x_, body_target_y_, posZ, absolute);
    body_cmd_sent_ = true;

    // RCLCPP_INFO(logger_, "[moveFoot:S1] leg=%d arrived2pos?=%d body_target_x=%.4f, body_target_y=%.4f posZ=%.4f",
    //             legID, legs_[legID]->arrived2pos(), body_target_x_, body_target_y_, posZ);

    // RCLCPP_INFO(logger_,
    //   "[moveFoot:S1->S2] leg=%d worldX=%.4f, worldY=%.4f cgX=%.4f cgY=%.4f cg_x=%.4f cg_y=%.4f desiredX=%.4f desiredY=%.4f tgtX=%.4f tgtY=%.4f incX=%.4f incY=%.4f body_tgt=(%.4f,%.4f)",
    //   legID, worldX, worldY, cgX, cgY, cg_x, cg_y, desiredX, desiredY, tgtX, tgtY, incX, incY, body_target_x_, body_target_y_);

    movementState = 2;
    stable_count_ = 0;
    return false;
  }

  // ---------------- STATE 2: esperar a que el BODY llegue (balancePlanner) ----------------
  if (movementState == 2)
  {
    if (body_cmd_sent_) {
      if (!arrived2pos())
        return false;

      stable_count_++;
      if (stable_count_ < body_stable_cycles)
        return false;
    }

    // body ya llegó (o no había comando)
    movementState = 3;
    stable_count_ = 0;
    return false;
  }

  // ---------------- STATE 3: FIN (cambia de pata sin checks) ----------------
  if (movementState == 3)
  {
    movementState = 4;
    return false;
  }

  if (movementState == 4)
  {
    centerPos();
    movementState = 0;
    active_leg_ = -1;
    stable_count_ = 0;
    return true;
  }

  // Fallback
  movementState = 0;
  active_leg_ = -1;
  stable_count_ = 0;
  return false;
}

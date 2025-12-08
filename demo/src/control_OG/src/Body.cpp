#include "Body.hpp"

Body::Body(const rclcpp::Node::SharedPtr& node)
: node_(node), logger_(node->get_logger()), balancePlanner(node->get_clock())
{}

void Body::set_body_size(float length_mm, float width_mm) {
  body_length = length_mm;
  body_width  = width_mm;
}

// void Body::set_neutral_stance(int x_front, int x_rear, int y_offset, int z_down){
//   // Coloca cada pie bajo su cadera con una altura razonable (negativa)
//   // Ajusta estos signos si tu URDF tiene distinto eje "abajo"
//   legs_[FR]->setFootAbsPos(+x_front, +y_offset, -std::abs(z_down), absolute);
//   legs_[FL]->setFootAbsPos(+x_front, -y_offset, -std::abs(z_down), absolute);
//   legs_[RR]->setFootAbsPos(-x_rear,  +y_offset, -std::abs(z_down), absolute);
//   legs_[RL]->setFootAbsPos(-x_rear,  -y_offset, -std::abs(z_down), absolute);
// }

// void Body::set_initial_joint_angles_deg(std::array<int,12>& d){
//   // Orden de 'd': FL(0..2), RL(3..5), RR(6..8), FR(9..11) — ver abajo cómo lo llenamos desde main
//   legs_[FL]->set_joint_angles_deg(d[0],  d[1],  d[2]);
//   legs_[RL]->set_joint_angles_deg(d[3],  d[4],  d[5]);
//   legs_[RR]->set_joint_angles_deg(d[6],  d[7],  d[8]);
//   legs_[FR]->set_joint_angles_deg(d[9],  d[10], d[11]);
// }

void Body::set_initial_joint_angles_deg(std::array<double,12>& d){
  // Orden de 'd': FL(0..2), RL(3..5), RR(6..8), FR(9..11) 

  std::array<double,3> qFL{{d[0], d[1], d[2]}};
  std::array<double,3> qRL{{d[3], d[4], d[5]}};
  std::array<double,3> qRR{{d[6], d[7], d[8]}};
  std::array<double,3> qFR{{d[9], d[10], d[11]}};

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

  // FR=0, RR=1, FL=2, RL=3 (orden de tu enum)
  constexpr std::array<std::pair<Side,End>, 4> kLegPose{{
    { Side::right, End::front }, // FR
    { Side::right, End::rear  }, // RR
    { Side::left,  End::front }, // FL
    { Side::left,  End::rear  }  // RL
  }};

  for (int i = 0; i < 4; ++i) {
    auto id = static_cast<LegID>(i);
    auto [side, end] = kLegPose[i];
    if (!legs_[i]) {
    //   legs_[i] = std::make_shared<Leg>(id, 40, 50, 50, side, end, node->get_logger());
        legs_[i] = std::make_shared<Leg>(id, 430, 200, 160, side, end, node->get_logger());
    }

    // Asignar nombres de joint consistentes con el orden [sh1, sh2, elbow]
    // Ej: "FR_sh1", "FR_sh2", "FR_elbow"
    const std::array<const char*, 4> leg_id_str{{"FR","RR","FL","RL"}};

    std::array<std::string, 3> jnames{{
        std::string(leg_id_str[i]) + "_sh1",
        std::string(leg_id_str[i]) + "_sh2",
        std::string(leg_id_str[i]) + "_elbow"
    }};
    legs_[i]->set_joint_names(jnames);

    // (Opcional) posiciones iniciales
    // std::array<double,3> init_deg = {0.0, 0.0, 0.0};
    // legs_[i]->set_joint_positions_deg(init_deg);
  }

  // Posición absoluta de pies según tamaño de cuerpo
  for (auto& l : legs_) {
    if (l) l->initAbsFootPos(body_length, body_width);
  }

  RCLCPP_INFO(logger_, "Body setup complete.");
}

void Body::setCamber(float camber_in_mm){
    float dCamber = camber_in_mm - camber;
    legs_[FR]->footY -= dCamber;
    legs_[RR]->footY -= dCamber;
    legs_[FL]->footY += dCamber;
    legs_[RL]->footY += dCamber;
    camber = camber_in_mm;
}

void Body::move(float dx, float dy, float dz){
    posX += dx;
    posY += dy;
    posZ += dz;
}

void Body::go2pos(float px, float py, float pz, int type){
    if (type == absolute){
        balancePlanner.setPlanner(body_speed, posX, posY, posZ, px, py, pz);
    }
    else if (type == incremental){
        balancePlanner.setPlanner(body_speed, posX, posY, posZ, posX+px, posY+py, posZ+pz);
    }
    isIdle = false;
}

void Body::updatePose(){

    float x_rel;
    float y_rel;
    float z_rel;
    float z_pitch;

    // float l0 = 40.0f;
    float l0 = 430.0f;
    // Front right
    x_rel = legs_[FR]->footX - body_length/2 - posX;
    y_rel = legs_[FR]->footY - body_width/2 - l0 - posY;
    z_rel = legs_[FR]->footZ - posZ;
    computeYaw(yaw, body_width, body_length, l0, &x_rel, &y_rel);
    computePitch(pitch, body_length, -z_rel, &x_rel, &z_pitch);
    computeRoll(-roll, body_width, l0, z_pitch, &y_rel, &z_rel);
    // RCLCPP_INFO(logger_, "FR: x rel: %f, y rel: %f, z rel: %f\n", x_rel, y_rel, z_rel);
    legs_[FR]->update(-x_rel, y_rel, -z_rel);

    // Rear right
    x_rel = legs_[RR]->footX + body_length/2 - posX;
    y_rel = legs_[RR]->footY - body_width/2 - l0 - posY;
    z_rel = legs_[RR]->footZ - posZ;
    computeYaw(yaw, body_width, -body_length, l0, &x_rel, &y_rel);
    computePitch(pitch, -body_length, -z_rel, &x_rel, &z_pitch);
    computeRoll(-roll, body_width, l0, z_pitch, &y_rel, &z_rel);
    // RCLCPP_INFO(logger_, "RR: x rel: %f, y rel: %f, z rel: %f\n", x_rel, y_rel, z_rel);
    legs_[RR]->update(-x_rel, y_rel, -z_rel);

    // Front left
    x_rel = legs_[FL]->footX - body_length/2 - posX;
    y_rel = legs_[FL]->footY + body_width/2 + l0 - posY;
    z_rel = legs_[FL]->footZ - posZ;
    computeYaw(yaw, -body_width, body_length, -l0, &x_rel, &y_rel);
    computePitch(pitch, body_length, -z_rel, &x_rel, &z_pitch);
    computeRoll(-roll, -body_width, -l0, z_pitch, &y_rel, &z_rel);
    // RCLCPP_INFO(logger_, "FL: x rel: %f, y rel: %f, z rel: %f\n", x_rel, y_rel, z_rel);
    legs_[FL]->update(-x_rel, y_rel, -z_rel);
    
    // Rear left
    x_rel = legs_[RL]->footX + body_length/2 - posX;
    y_rel = legs_[RL]->footY + body_width/2 + l0 - posY;
    z_rel = legs_[RL]->footZ - posZ;
    computeYaw(yaw, -body_width, -body_length, -l0, &x_rel, &y_rel);
    computePitch(pitch, -body_length, -z_rel, &x_rel, &z_pitch);
    computeRoll(-roll, -body_width, -l0, z_pitch, &y_rel, &z_rel);
    // RCLCPP_INFO(logger_, "RL: x rel: %f, y rel: %f, z rel: %f\n", x_rel, y_rel, z_rel);
    legs_[RL]->update(-x_rel, y_rel, -z_rel);
}

// void Body::updatePose()
// {
//   // Imprime cada N ciclos para no spamear
//   static uint32_t tick = 0;
//   constexpr uint32_t kPrintEvery = 10; // imprime 1 de cada 10 ciclos
//   const bool do_print = ((tick++ % kPrintEvery) == 0);

//   const float L  = static_cast<float>(body_length);
//   const float W  = static_cast<float>(body_width);
//   const float l0 = 40.0f;

//   auto log_header = [&](){
//     if (!do_print) return;
//     RCLCPP_INFO(logger_, "==== updatePose() ====");
//     RCLCPP_INFO(logger_, "Body pose  (world): posX=%.3f posY=%.3f posZ=%.3f | roll=%.3f pitch=%.3f yaw=%.3f (rad)",
//                 posX, posY, posZ, roll, pitch, yaw);
//     RCLCPP_INFO(logger_, "Body dims: L=%.3f W=%.3f l0=%.3f", L, W, l0);
//   };

//   auto worldToBody = [&](const char* leg_name,
//                          float fx, float fy, float fz,
//                          float shoulder_x, float shoulder_y,
//                          float* out_x, float* out_y, float* out_z)
//   {
//     float vx = fx - posX;
//     float vy = fy - posY;
//     float vz = fz - posZ;

//     if (do_print) {
//       RCLCPP_INFO(logger_, "[%s] foot(world): fx=%.3f fy=%.3f fz=%.3f", leg_name, fx, fy, fz);
//       RCLCPP_INFO(logger_, "[%s] vec(world):  vx=%.3f vy=%.3f vz=%.3f", leg_name, vx, vy, vz);
//     }

//     // R^-1: Z(-yaw) -> X(-pitch) -> Y(-roll)
//     rotYaw(-yaw, &vx, &vy);
//     if (do_print) RCLCPP_INFO(logger_, "[%s] after  Rz(-yaw):  vx=%.3f vy=%.3f vz=%.3f", leg_name, vx, vy, vz);

//     rotPitch(-pitch, &vx, &vz);
//     if (do_print) RCLCPP_INFO(logger_, "[%s] after  Rx(-pitch):vx=%.3f vy=%.3f vz=%.3f", leg_name, vx, vy, vz);

//     rotRoll(-roll, &vy, &vz);
//     if (do_print) RCLCPP_INFO(logger_, "[%s] after  Ry(-roll): vx=%.3f vy=%.3f vz=%.3f", leg_name, vx, vy, vz);

//     // traslada del centro del torso al hombro de esa pata
//     vx -= shoulder_x;
//     vy -= shoulder_y;

//     if (do_print) {
//       RCLCPP_INFO(logger_, "[%s] shoulder (body): sx=%.3f sy=%.3f", leg_name, shoulder_x, shoulder_y);
//       RCLCPP_INFO(logger_, "[%s] rel to shoulder: x_rel=%.3f y_rel=%.3f z_rel=%.3f", leg_name, vx, vy, vz);
//     }

//     *out_x = vx; *out_y = vy; *out_z = vz;
//   };

//   log_header();

//   // hombros en marco del cuerpo
//   const float sx_FR = +L*0.5f, sy_FR = -(W*0.5f + l0);
//   const float sx_RR = -L*0.5f, sy_RR = -(W*0.5f + l0);
//   const float sx_FL = +L*0.5f, sy_FL = +(W*0.5f + l0);
//   const float sx_RL = -L*0.5f, sy_RL = +(W*0.5f + l0);

//   float x_rel, y_rel, z_rel;

//   // -------- FR --------
//   worldToBody("FR",
//               legs_[FR]->footX, legs_[FR]->footY, legs_[FR]->footZ,
//               sx_FR, sy_FR, &x_rel, &y_rel, &z_rel);

//   // Si tu IK usa z hacia abajo positiva, descomenta:
//   // z_rel = -z_rel;
//   if (do_print) RCLCPP_INFO(logger_, "[FR] >> update(x=%.3f, y=%.3f, z=%.3f)", x_rel, y_rel, z_rel);
//   legs_[FR]->update(x_rel, y_rel, z_rel);

//   // -------- RR --------
//   worldToBody("RR",
//               legs_[RR]->footX, legs_[RR]->footY, legs_[RR]->footZ,
//               sx_RR, sy_RR, &x_rel, &y_rel, &z_rel);
//   // z_rel = -z_rel;
//   if (do_print) RCLCPP_INFO(logger_, "[RR] >> update(x=%.3f, y=%.3f, z=%.3f)", x_rel, y_rel, z_rel);
//   legs_[RR]->update(x_rel, y_rel, z_rel);

//   // -------- FL --------
//   worldToBody("FL",
//               legs_[FL]->footX, legs_[FL]->footY, legs_[FL]->footZ,
//               sx_FL, sy_FL, &x_rel, &y_rel, &z_rel);
//   // z_rel = -z_rel;
//   if (do_print) RCLCPP_INFO(logger_, "[FL] >> update(x=%.3f, y=%.3f, z=%.3f)", x_rel, y_rel, z_rel);
//   legs_[FL]->update(x_rel, y_rel, z_rel);

//   // -------- RL --------
//   worldToBody("RL",
//               legs_[RL]->footX, legs_[RL]->footY, legs_[RL]->footZ,
//               sx_RL, sy_RL, &x_rel, &y_rel, &z_rel);
//   // z_rel = -z_rel;
//   if (do_print) RCLCPP_INFO(logger_, "[RL] >> update(x=%.3f, y=%.3f, z=%.3f)", x_rel, y_rel, z_rel);
//   legs_[RL]->update(x_rel, y_rel, z_rel);
// }



bool Body::arrived2pos(){
    int isDone = balancePlanner.computeNewPos();
    posX = balancePlanner.newX;
    posY = balancePlanner.newY;
    posZ = balancePlanner.newZ;
    if (isDone){
        isIdle = true;
        return true;
    }
    return false;
}

void Body::getMeanFootPos(float *x, float *y, float *z){
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    for (int i = 0; i < 4; i++){
        sumX += legs_[i]->footX;
        sumY += legs_[i]->footY;
        sumZ += legs_[i]->footZ;
    }
    *x = sumX / 4;
    *y = sumY / 4;
    *z = sumZ / 4;
}

bool Body::moveFoot(int legID, float px, float py, float pz, int dz, int movementType, bool return2zero){
    if (movemetState == 0){
        //compute triangle CG
        float cgX = 0.0f, cgY = 0.0f;
        for (int i = 0; i < 4; i++){
            if (i != legID){
                cgX += legs_[i]->footX;
                cgY += legs_[i]->footY;
            }
        }
        cgX = cgX / 3;
        cgY = cgY / 3;

        //body correction
        float dX, dY;
        dX = posX;
        if (legID == RR || legID == RL){
            dX = (legs_[FL]->footX + legs_[FR]->footX) / 2 - 80;
        }
        if (legID == FR || legID == RR){
            dY = (legs_[FL]->footY + legs_[RL]->footY) / 2 + 30;
        }
        else{
            dY = (legs_[FR]->footY + legs_[RR]->footY) / 2 - 30;
        }

        float alpha = 0.5;
        float bodyX = cgX*alpha + dX*(1-alpha);
        float bodyY = cgY*alpha + dY*(1-alpha);

        go2pos(bodyX, bodyY, posZ, absolute);
        isIdle = false;
        movemetState = 1;
    }
    else if (movemetState == 1){
        if (arrived2pos()){
            legs_[legID]->go2pos(px, py, pz, dz, movementType);
            movemetState = 2;
        }
    }
    else if (movemetState == 2){
        if (legs_[legID]->arrived2pos()){
            if (return2zero){
                float x, y, z;
                getMeanFootPos(&x, &y, &z);
                go2pos(x, y, posZ, absolute);
                movemetState = 3;
            }
            else{
                movemetState = 0;
                isIdle = true;
                return true;
            }
        }
    }
    else if (movemetState == 3){
        if (arrived2pos()){
            movemetState = 0;
            isIdle = true;
            return true;
        }
    }
    return false;
}

bool Body::centerPos(float dx, float dy, float dz){
    if (centeringState == 0){
        float x, y, z;
        getMeanFootPos(&x, &y, &z);
        go2pos(x + dx, y + dy, z + dz, absolute);
        centeringState = 1;
    }
    else if (centeringState == 1){
        if (arrived2pos()){
            centeringState = 0;
            return true;
        }
    }
    return false;
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
    const std::string& sh2   = names[base + 1];
    const std::string& sh1   = names[base + 2];
    legs_[id]->set_joint_names(sh1, sh2, elbow);
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

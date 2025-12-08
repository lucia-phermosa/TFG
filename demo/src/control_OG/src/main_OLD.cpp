#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <vector>
#include <string>
#include <array>
#include <algorithm>
#include <atomic>
#include <cmath>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include "Body.hpp"
#include "Gaits.hpp"
#include "Horizontal_controller.hpp"
#include "keyboard_cmd_parser.hpp"

// === IMPORTANTE: usar TFMessage (ros_gz) ===
#include <tf2_msgs/msg/tf_message.hpp>

using namespace std::chrono_literals;
using FJT = control_msgs::action::FollowJointTrajectory;

class JointPubNode : public rclcpp::Node {
public:
  JointPubNode() : Node("joint_pub_node") {
    // Parámetros
    declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<double>("start_after_seconds", 0.0);
    declare_parameter<int64_t>("socket_port", socket_port_);
    declare_parameter<std::string>("controller_name", "joint_trajectory_controller");
    declare_parameter<double>("goal_degrees", 15.0);
    declare_parameter<double>("goal_time_s", 1.0);
    declare_parameter<bool>("publish_joint_states", false);
    declare_parameter<double>("trajectory_lookahead_s", 0.05);
    declare_parameter<std::string>("imu_topic", imu_topic_);

    // Nombre real del modelo en Gazebo y tópico de pose bridged
    declare_parameter<std::string>("model_name", model_name_);
    declare_parameter<std::string>("model_pose_topic", model_pose_topic_);

    auto names        = get_parameter("joint_names").as_string_array();
    const double hz   = get_parameter("publish_rate_hz").as_double();
    start_delay_s_    = get_parameter("start_after_seconds").as_double();
    socket_port_      = get_parameter("socket_port").as_int();
    controller_name_  = get_parameter("controller_name").as_string();
    goal_deg_         = get_parameter("goal_degrees").as_double();
    goal_time_s_      = get_parameter("goal_time_s").as_double();
    publish_js_       = get_parameter("publish_joint_states").as_bool();
    traj_lookahead_s_ = get_parameter("trajectory_lookahead_s").as_double();
    imu_topic_        = get_parameter("imu_topic").as_string();
    model_name_       = get_parameter("model_name").as_string();
    model_pose_topic_ = get_parameter("model_pose_topic").as_string();

    if (model_name_.empty()) {
      // pon aquí tu nombre real por defecto
      model_name_ = "robot"; 
    }
    if (model_pose_topic_.empty()) {
      model_pose_topic_ = "/model/" + model_name_ + "/pose";
    }

    if (names.size() != 12) {
      RCLCPP_WARN(get_logger(), "Esperaba 12 joint_names, recibidos: %zu.", names.size());
    }

    // Body / Gait
    auto self = rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){});
    body_ = std::make_shared<Body>(self);
    body_->set_body_size(120.0f, 80.0f);
    body_->setup();
    body_->assign_joint_names_from_array(names);

    for (size_t i=0; i<body_->legs().size(); ++i) {
      RCLCPP_INFO(get_logger(), "leg[%zu]: %s", i, body_->legs()[i] ? "OK" : "NULL");
    }
    if (body_->legs().size() < 4 || 
        !body_->legs()[0] || !body_->legs()[1] || !body_->legs()[2] || !body_->legs()[3]) {
      RCLCPP_FATAL(get_logger(), "Body no inicializó todas las patas. Revisa Body::setup()/assign_joint_names_from_array().");
      throw std::runtime_error("legs not constructed");
    }

    int i = 0;
    for (auto& leg_ptr : body_->legs()) {
      if (leg_ptr) {
        RCLCPP_INFO(this->get_logger(), "Leg Side: %d, End: %d",
                    static_cast<int>(leg_ptr->get_side()),
                    static_cast<int>(leg_ptr->get_end()));
      }
      i++;
    }

    horiz_control.init();
    gait_.setBody(body_.get());
    // gait_.setSpeed(20.0, 0.0);
    // gait_.pause();

    if (publish_js_) {
      js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }

    // Publisher de JointTrajectory para ros2_control
    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        controller_name_ + std::string("/joint_trajectory"), 10);

    // Orden de nombres de joints para las trayectorias
    joint_names_.clear();
    for (auto& leg : body_->legs()) {
      if (!leg) continue;
      const auto& n = leg->joint_names();
      for (int j=0; j<3; ++j) {
        if (!n[j].empty()) joint_names_.push_back(n[j]);
      }
    }
    if (joint_names_.empty() && !names.empty()) {
      joint_names_.assign(names.begin(), names.end());
    }

    // IMU
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg){
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);  // rad
        imu_roll_  = r;
        imu_pitch_ = p;
        imu_yaw_   = y;
      }
    );
    RCLCPP_INFO(get_logger(), "Suscrito a IMU en: %s", imu_topic_.c_str());

    // === POSE DEL MODELO DESDE GAZEBO (TFMessage) ===
    // QoS best effort porque viene de bridge
    model_pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      model_pose_topic_,
      rclcpp::QoS(10).best_effort(),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg){
        // PosePublisher publica varias transforms; buscamos la del modelo
        for (const auto &t : msg->transforms) {
          const std::string &child = t.child_frame_id;
          // Igualdad exacta o sufijo (por si incluye el scope del mundo)
          if (child == model_name_ || 
              (child.size() >= model_name_.size() &&
               child.compare(child.size()-model_name_.size(), model_name_.size(), model_name_)==0)) {
            current_x_.store(t.transform.translation.x);
            current_y_.store(t.transform.translation.y);
            current_z_.store(t.transform.translation.z);
            break;
          }
        }
      }
    );
    RCLCPP_INFO(get_logger(), "Suscrito a pose del modelo en: %s (modelo: %s)",
                model_pose_topic_.c_str(), model_name_.c_str());

    // Servidor TCP
    openSocketAndListen();

    start_time_ = now() + rclcpp::Duration::from_seconds(start_delay_s_);
    RCLCPP_INFO(get_logger(), "Arranque diferido: %.2f s.", start_delay_s_);

    // Timers
    cmd_timer_ = create_wall_timer(10ms, [this](){ this->processSocket(); });

    auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, hz));
    cycle_dt_ = period;
    main_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this](){ this->tick(); }
    );
  }

  ~JointPubNode() override {
    if (conn_fd_   >= 0) { ::close(conn_fd_);   conn_fd_ = -1; }
    if (listen_fd_ >= 0) { ::close(listen_fd_); listen_fd_ = -1; }
  }

private:
  void processSocket() {
    // aceptar si no hay conexión aún
    if (conn_fd_ < 0 && listen_fd_ >= 0) {
      sockaddr_in caddr{}; socklen_t clen = sizeof(caddr);
      int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&caddr), &clen);
      if (fd >= 0) {
        int f = fcntl(fd, F_GETFL, 0); fcntl(fd, F_SETFL, f | O_NONBLOCK);
        conn_fd_ = fd;
        parser_ = std::make_unique<CommandParser>(conn_fd_, this->get_logger());
        RCLCPP_INFO(this->get_logger(), "Cliente conectado.");
      } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        RCLCPP_ERROR(this->get_logger(), "accept(): %s", strerror(errno));
      }
    }

    if (!parser_) return;

    while (parser_->readNext()) {
      const int code = parser_->last_code();
      const std::string &label = parser_->last_label();

      if (code == 1) { // XY -> destino absoluto en el mundo
        if (!walk_ON){
          const double targetX = parser_->pX;
          const double targetY = parser_->pY;
          const double targetZ = parser_->pZ;

          // Pose actual de Gazebo:
          const double curx = current_x_.load();
          const double cury = current_y_.load();
          const double curz = current_z_.load();

          // Offset a aplicar por tu máquina de estados
          robotX = static_cast<float>(targetX - curx);
          robotY = static_cast<float>(targetY - cury);
          robotZ = static_cast<float>(targetZ - curz);

          newPose = true;
          RCLCPP_INFO(this->get_logger(),
            "Destino (%.3f, %.3f, %.3f). Actual (%.3f, %.3f, %.3f). Offset (%.3f, %.3f, %.3f)",
            targetX, targetY, targetZ, curx, cury, curz, robotX, robotY, robotZ);
        }
      }
      else if (code == 3) { // WL
        RCLCPP_INFO(this->get_logger(), "Walk command");
        walk_ON = (parser_->walkMode == 1);
        if (walk_ON) { if (gait_.isPaused) gait_.resume(); }
        else { gait_.pause(); }
      }
      else if (code == 4) { // OF
        const int leg_id = parser_->leg_id;
        RCLCPP_INFO(this->get_logger(), "[OF] set_joint_offset() no existe en Leg (leg=%d, joint=%d, off=%d)",
                    leg_id, parser_->joint % 3, parser_->joint_offset);
        newPose = true;
      }
      else if (code == 5) { // AG
        const int leg_id = parser_->leg_id;
        RCLCPP_INFO(this->get_logger(), "[AG] setServoAngle() no existe en Leg (leg=%d, joint=%d, angle=%d)",
                    leg_id, parser_->joint % 3, parser_->joint_angle);
      }
      else if (code == 6) { // LI
        const int leg_id = parser_->leg_id;
        RCLCPP_INFO(this->get_logger(), "[LI] servo_saturation.setLimits() no existe en Leg (leg=%d, joint=%d, [%d,%d])",
                    leg_id, parser_->joint % 3, parser_->lowerLimit, parser_->upperLimit);
      }
      else if (code == 7) { // CD
        RCLCPP_INFO(this->get_logger(), "[CD] sendCalibData() no existe en Leg (se omite).");
      }
      else if (code == 8) { // PI
        pitch_cmd = (float)parser_->pitch / 180.0f * (float)M_PI;
        newPose = true;
      }
      else if (code == 9) { // RO
        roll_cmd = (float)parser_->roll / 180.0f * (float)M_PI;
        newPose = true;
      }
      else if (code == 10) { // YW
        yaw = (float)parser_->yaw / 180.0f * (float)M_PI;
        newPose = true;
      }
      else if (code == 11) { // TL
        telem_ON = (parser_->telem == 1);
      }
      else if (code == 12) { // HZ
        horiz_control.hold_ON = (parser_->hold_orient == 1);
        horiz_control.resetControllers();
        newPose = true;
      }
      else if (code == 13) { // CG
        horiz_control.cg_control = (parser_->cg_control == 1);
        newPose = true;
      }
      else if (code == 14) { // FP
        const int leg_id = parser_->leg_id;
        RCLCPP_INFO(this->get_logger(), "[FP] setFootPos() no encontrado en Leg (leg=%d, %f,%f,%f)",
                    leg_id, parser_->pX, parser_->pY, parser_->pZ);
        newPose = true;
      }
      else if (code == 15) { // CA
        body_->setCamber(parser_->camber);
        newPose = true;
      }
      else if (code == 16) { // VE
        if (walk_ON) {
          gait_.setSpeed(static_cast<float>(parser_->velX), static_cast<float>(parser_->velY));
          if (std::abs(parser_->velX) > 10 || std::abs(parser_->velY) > 10) {
            if (gait_.isPaused) gait_.resume();
          } else {
            gait_.pause();
          }
        }
      }
      else if (code == 17) { // TR
        trot_gait   = (parser_->trot_gait == 1);
        trot_state  = (trot_gait ? 1 : 0);
        trot_start  = this->now();
        trot_t_last_s = 0.0f;
        t_sim = 0.0f;
        RCLCPP_INFO(this->get_logger(), "Trot command");
      }
      else if (code == 18) { // OS
        osc_amp = parser_->osc_amp;
        osc_w   = parser_->osc_w / 10.0f;
        osc_phi = parser_->osc_phi / 10.0f;
      }
      else if (code == 19) { // OX
        oscX_amp = parser_->oscX_amp;
        oscY_amp = parser_->oscY_amp;
      }
      else if (code == 20) { // OP
        oscX_phi = parser_->oscX_phi / 10.0f;
        oscY_phi = parser_->oscY_phi / 10.0f;
      }
      else if (code == 21) { // RY
        imu_yaw_.store(0.0);
      }
      else if (code == -1) { // R
        gait_.pause();
        walk_ON = false;
        centerBody = true;
        isMoving = false;
        RCLCPP_INFO(this->get_logger(), "[R] Reset -> pause + centrar cuerpo");
      }
      else {
        RCLCPP_WARN(this->get_logger(), "Comando desconocido: #%s; (code=%d)",
                    label.c_str(), code);
      }
    }
  }

  void tick() {
    if (!started_) {
      if (now() >= start_time_) {
        // gait_.resume(); 
        started_ = true;
        RCLCPP_INFO(this->get_logger(), "¡Comenzando a andar!");
      }
    } 
    else {
        float imu_roll_deg  = static_cast<float>(imu_roll_.load()  * 180.0 / M_PI);
        float imu_pitch_deg = static_cast<float>(imu_pitch_.load() * 180.0 / M_PI);

        if (!isMoving && !trot_gait){
            float x, y, z;
            body_->getMeanFootPos(&x, &y, &z);
            // Posición actual de Gazebo + offset pedido por code==1
            const double curx = current_x_.load();
            const double cury = current_y_.load();
            const double curz = current_z_.load();
            body_->posX = static_cast<float>(curx) + robotX + x;
            body_->posY = static_cast<float>(cury) + robotY + y;
            body_->posZ = static_cast<float>(curz) + robotZ + z;
        }

        horiz_control.newRobotOrient(
        static_cast<float>( body_->roll * 180.0 / M_PI ),
        static_cast<float>( body_->pitch * 180.0 / M_PI ),
        static_cast<float>(body_->posZ),
        static_cast<float>(-imu_roll_deg),
        static_cast<float>( imu_pitch_deg)
        );

        if (horiz_control.cg_control) {
            body_->move(horiz_control.cg_x, -horiz_control.cg_y, 0);
            newPose = true;
        }

        if (horiz_control.hold_ON) {
            float roll_deg_out = 0.0f, pitch_deg_out = 0.0f;
            float roll_cmd_deg  = static_cast<float>(roll_cmd  * 180.0 / M_PI);
            float pitch_cmd_deg = static_cast<float>(pitch_cmd * 180.0 / M_PI);

            horiz_control.stabilize(
                roll_cmd_deg,
                pitch_cmd_deg,
                static_cast<float>(-imu_roll_deg),
                static_cast<float>( imu_pitch_deg),
                &roll_deg_out,
                &pitch_deg_out
            );
        } 
        else {
            roll  = roll_cmd;
            pitch = pitch_cmd;
        }

        body_->roll  = roll;
        body_->pitch = pitch;
        body_->yaw   = yaw;

        // printf("Roll: %f, Pitch: %f, Yaw: %f\n", body_->roll, body_->pitch, body_->yaw);

        if (newPose){
            body_->updatePose();
            newPose = false;
        }

        if (walk_ON && !gait_.hasFinished()){
          bool hasFinish = gait_.walk(); (void)hasFinish;

          if (gait_.isPaused && body_->isIdle) {
            centerBody = true;
          } else {
            isMoving = true;
          }
          newPose = true;
        }

        if (centerBody){
          bool centerDone = body_->centerPos(0, 0, body_->posZ);
          isMoving = true;
          if (centerDone){
            centerBody = false;
            isMoving = false;
          }
          newPose = true;
        }

        if (trot_gait){
          float t  = static_cast<float>((this->now() - trot_start).seconds());
          float dt = t - trot_t_last_s;
          trot_t_last_s = t;

          t_sim += dt * osc_w;

          float dZ = static_cast<float>(osc_amp * std::sin(t_sim));
          dZ = (dZ < 0) ? 0 : dZ;
          body_->legs_[FR]->footZ = dZ;
          body_->legs_[RL]->footZ = dZ;

          float dX = static_cast<float>(oscX_amp * std::sin(t_sim - static_cast<float>(M_PI)/2.0f + oscX_phi));
          body_->legs_[FR]->footX = dX + body_->body_length/2;
          body_->legs_[RL]->footX = dX - body_->body_length/2;

          float dY = static_cast<float>(oscY_amp * std::sin(t_sim - static_cast<float>(M_PI)/2.0f + oscY_phi));
          body_->legs_[FR]->footY = dY + body_->body_width/2 + 40;
          body_->legs_[RL]->footY = dY - body_->body_width/2 - 40;

          dZ = static_cast<float>(osc_amp * std::sin(t_sim + osc_phi));
          dZ = (dZ < 0) ? 0 : dZ;
          body_->legs_[FL]->footZ = dZ;
          body_->legs_[RR]->footZ = dZ;

          dX = static_cast<float>(oscX_amp * std::sin(t_sim + osc_phi - static_cast<float>(M_PI)/2.0f + oscX_phi));
          body_->legs_[FL]->footX = dX + body_->body_length/2;
          body_->legs_[RR]->footX = dX - body_->body_length/2;

          dY = static_cast<float>(oscY_amp * std::sin(t_sim + osc_phi - static_cast<float>(M_PI)/2.0f + oscY_phi));
          body_->legs_[FL]->footY = dY - body_->body_width/2 - 40;
          body_->legs_[RR]->footY = dY + body_->body_width/2 + 40;

          // Mantén posX/posY si usas offset aparte durante trote
          // (ajústalo a tu lógica si quieres que el trote también vaya al destino)
          newPose = true;
        }
    }

    // JointState opcional
    if (publish_js_ && js_pub_) {
      sensor_msgs::msg::JointState js;
      js.header.stamp = now();
      for (auto& leg : body_->legs()) {
        if (!leg) continue;
        const auto& names = leg->joint_names();
        const auto  pos   = leg->joint_positions_rad();
        for (int j = 0; j < 3; ++j) {
          if (!names[j].empty()) { js.name.push_back(names[j]); js.position.push_back(pos[j]); }
        }
      }
      js_pub_->publish(js);
    }

    // Stream de trayectorias a ros2_control
    if (traj_pub_) {
      std::vector<double> positions; 
      positions.reserve(joint_names_.size());

      for (auto& leg : body_->legs()) {
        if (!leg) continue;
        const auto pos = leg->joint_positions_rad();
        for (int j = 0; j < 3; ++j)
          positions.push_back(pos[j]);
      }

      if (positions.size() != joint_names_.size()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Tamaño de posiciones (%zu) != joint_names (%zu). No se publica trayectoria.",
                            positions.size(), joint_names_.size());
        return;
      }

      trajectory_msgs::msg::JointTrajectory jt;
      jt.header.stamp = now();
      jt.joint_names  = joint_names_;

      trajectory_msgs::msg::JointTrajectoryPoint p;
      const double dt = std::max(1e-3, traj_lookahead_s_);
      p.time_from_start = rclcpp::Duration::from_seconds(dt);
      p.positions = positions;
      p.velocities.resize(positions.size(), 0.0);

      jt.points.push_back(std::move(p));

      // === 🟢 DEBUG PRINT: valores publicados ===
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3);
      oss << "[Joint positions] ";
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        oss << joint_names_[i] << "=" << positions[i];
        if (i < joint_names_.size() - 1) oss << ", ";
      }
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,  // imprime 1 vez/seg
          "%s", oss.str().c_str());
      // ==========================================

      traj_pub_->publish(jt);
    }
    // if (traj_pub_) {
    //   std::vector<double> positions; positions.reserve(joint_names_.size());
    //   for (auto& leg : body_->legs()) {
    //     if (!leg) continue;
    //     const auto pos = leg->joint_positions_rad();
    //     for (int j=0; j<3; ++j) positions.push_back(pos[j]);
    //   }
    //   if (positions.size() != joint_names_.size()) {
    //     RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
    //                          "Tamaño de posiciones (%zu) != joint_names (%zu). No se publica trayectoria.",
    //                          positions.size(), joint_names_.size());
    //     return;
    //   }

    //   trajectory_msgs::msg::JointTrajectory jt;
    //   jt.header.stamp = now();
    //   jt.joint_names  = joint_names_;

    //   trajectory_msgs::msg::JointTrajectoryPoint p;
    //   const double dt = std::max(1e-3, traj_lookahead_s_);
    //   p.time_from_start = rclcpp::Duration::from_seconds(dt);
    //   p.positions = positions;
    //   p.velocities.resize(positions.size(), 0.0);

    //   jt.points.push_back(std::move(p));
    //   traj_pub_->publish(jt);
    // }
  }

  void openSocketAndListen() {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "socket(): %s", strerror(errno));
      return;
    }
    int opt = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(socket_port_));
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "bind(): %s", strerror(errno));
      ::close(listen_fd_); listen_fd_ = -1; return;
    }
    if (::listen(listen_fd_, 1) < 0) {
      RCLCPP_ERROR(get_logger(), "listen(): %s", strerror(errno));
      ::close(listen_fd_); listen_fd_ = -1; return;
    }
    int f = fcntl(listen_fd_, F_GETFL, 0); fcntl(listen_fd_, F_SETFL, f | O_NONBLOCK);
    RCLCPP_INFO(get_logger(), "Servidor TCP escuchando en 0.0.0.0:%ld", socket_port_);
  }

private:
  std::shared_ptr<Body> body_;
  WalkGait gait_;
  Horizontal_controller horiz_control;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  double start_delay_s_ = 10.0;
  rclcpp::Time start_time_;
  bool started_ = false;

  bool walk_ON = false;
  bool holdOrient = false;

  bool trot_gait = false;
  int  trot_state = 0;
  rclcpp::Time trot_start;
  float trot_t_last_s = 0.0f;
  float last_time = 0;
  float osc_amp = 0;
  float osc_w = 0;
  float osc_phi = 0;
  float yaw_corr = 0;
  float t_sim = 0;

  float oscX_amp = 0;
  float oscX_phi = 0;
  float oscY_amp = 0;
  float oscY_phi = 0;

  bool  newPose = false;
  float roll = 0, pitch = 0, yaw = 0;
  float robotX = 0.0f, robotY = 0.0f, robotZ = 0.0f;

  float roll_cmd = 0.0f;
  float pitch_cmd = 0.0f;

  bool telem_ON = false;
  bool isMoving = false;
  bool centerBody = false;

  // IMU
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::string imu_topic_{"/imu"};
  std::atomic<double> imu_roll_{0.0}, imu_pitch_{0.0}, imu_yaw_{0.0};

  // Pose del modelo (TFMessage)
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr model_pose_sub_;
  std::string model_name_{"/robot"};
  std::string model_pose_topic_{};

  std::atomic<double> current_x_{0.0}, current_y_{0.0}, current_z_{0.0};

  std::vector<std::string> joint_names_;
  std::chrono::duration<double> cycle_dt_{0.02};
  double traj_lookahead_s_{0.05};

  int64_t socket_port_ = 9000;
  std::string controller_name_{"joint_trajectory_controller"};
  double goal_deg_{15.0};
  double goal_time_s_{1.0};
  bool publish_js_{false};

  int listen_fd_ = -1;
  int conn_fd_   = -1;
  std::unique_ptr<CommandParser> parser_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointPubNode>());
  rclcpp::shutdown();
  return 0;
}

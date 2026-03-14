#include "JointPubNode.hpp"

using namespace std::chrono_literals;

JointPubNode::JointPubNode()
: Node("joint_pub_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Parámetros
  declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
  declare_parameter<double>("publish_rate_hz", 50.0);
  declare_parameter<double>("start_after_seconds", 0.0);
  declare_parameter<int64_t>("socket_port", socket_port_);
  declare_parameter<std::string>("controller_name", "joint_trajectory_controller");
  declare_parameter<bool>("publish_joint_states", false);
  declare_parameter<double>("trajectory_lookahead_s", 0.05);
  declare_parameter<std::string>("imu_topic", imu_topic_);
  declare_parameter<std::string>("odom_topic", odom_topic_);
  declare_parameter<std::string>("model_name", model_name_);

  auto names        = get_parameter("joint_names").as_string_array();
  const double hz   = get_parameter("publish_rate_hz").as_double();
  start_delay_s_    = get_parameter("start_after_seconds").as_double();
  socket_port_      = get_parameter("socket_port").as_int();
  controller_name_  = get_parameter("controller_name").as_string();
  publish_js_       = get_parameter("publish_joint_states").as_bool();
  traj_lookahead_s_ = get_parameter("trajectory_lookahead_s").as_double();
  imu_topic_        = get_parameter("imu_topic").as_string();
  odom_topic_       = get_parameter("odom_topic").as_string();
  model_name_       = get_parameter("model_name").as_string();

  if (model_name_.empty()) {
    model_name_ = "demo";
  }

  tf_timer_ = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());

  tf_buffer_.setCreateTimerInterface(tf_timer_);

  // Timer SOLO para inicializar feet desde TF
  init_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&JointPubNode::initTF, this)
  );

  RCLCPP_INFO(get_logger(), "Node constructed, waiting for TF...");

  if (names.size() != 12) {
    RCLCPP_WARN(get_logger(), "Esperaba 12 joint_names, recibidos: %zu.", names.size());
  }

  // Body / Gait
  auto self = rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){});
  body_ = std::make_shared<Body>(self);
  body_->set_body_size(0.2f, 0.189f);
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

  horiz_control.init();
  gait_.setBody(body_.get());

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
      tf2::Matrix3x3(q).getRPY(r, p, y);
      imu_roll_  = r;
      imu_pitch_ = p;
    }
  );
  RCLCPP_INFO(get_logger(), "Suscrito a IMU en: %s", imu_topic_.c_str());

  // ODOM
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(50),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      odom_x_.store(msg->pose.pose.position.x);
      odom_y_.store(msg->pose.pose.position.y);
      odom_z_.store(msg->pose.pose.position.z);

      tf2::Quaternion q;
      tf2::fromMsg(msg->pose.pose.orientation, q);
      double r,p,y;
      tf2::Matrix3x3(q).getRPY(r,p,y);
      odom_yaw_.store(y);
    }
  );
  RCLCPP_INFO(get_logger(), "Suscrito a ODOM en: %s", odom_topic_.c_str());

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

  if (logging_active3_ && !csv_file3_.is_open()) {
    RCLCPP_WARN(get_logger(), "[CSV] Opening trot_telemetry_1.csv");
    csv_file3_.open("/home/lucia/Desktop/videos/csv/roll_FUZZY2.csv", std::ios::out | std::ios::trunc);
    csv_file3_ << "timestamp, roll, cg_x, cg_y\n";
    if (!csv_file3_.is_open()) {
      RCLCPP_ERROR(get_logger(), "[CSV] Failed to open csv/trot_telemetry_1.csv");
    }
  }
}

JointPubNode::~JointPubNode() {
  if (conn_fd_   >= 0) { ::close(conn_fd_);   conn_fd_ = -1; }
  if (listen_fd_ >= 0) { ::close(listen_fd_); listen_fd_ = -1; }
  csv_file3_.close();
  csv_file2_.close();
  csv_file_.close();
}

// ======================= TF / OFFSETS =======================

inline Vec3 JointPubNode::tfTransToVec3(const geometry_msgs::msg::TransformStamped& T)
{
  return Vec3{
    static_cast<float>(T.transform.translation.x),
    static_cast<float>(T.transform.translation.y),
    static_cast<float>(T.transform.translation.z)
  };
}

bool JointPubNode::readLegOffsetsFromTF(tf2_ros::Buffer& tf_buffer,
                                        const std::string& hombro,
                                        const std::string& brazo,
                                        const std::string& antebrazo,
                                        const std::string& foot,
                                        LegOffsets *out,
                                        rclcpp::Logger logger,
                                        rclcpp::Clock::SharedPtr clock)
{
  const bool ok =
    tf_buffer.canTransform(hombro, brazo, tf2::TimePointZero) &&
    tf_buffer.canTransform(brazo, antebrazo, tf2::TimePointZero) &&
    tf_buffer.canTransform(antebrazo, foot, tf2::TimePointZero);

  if (!ok) {
    RCLCPP_INFO_THROTTLE(logger, *clock, 1000,
      "Waiting TF for offsets: %s->%s, %s->%s, %s->%s",
      hombro.c_str(), brazo.c_str(),
      brazo.c_str(), antebrazo.c_str(),
      antebrazo.c_str(), foot.c_str());
    return false;
  }

  auto Tsh = tf_buffer.lookupTransform(hombro, brazo, tf2::TimePointZero);
  auto Tel = tf_buffer.lookupTransform(brazo, antebrazo, tf2::TimePointZero);
  auto Tft = tf_buffer.lookupTransform(antebrazo, foot, tf2::TimePointZero);

  out->t_sh = tfTransToVec3(Tsh);
  out->t_el = tfTransToVec3(Tel);
  out->t_ft = tfTransToVec3(Tft);
  out->valid = true;

  RCLCPP_INFO(logger,
    "[OFFSETS] %s: t_sh=(%.6f %.6f %.6f) t_el=(%.6f %.6f %.6f) t_ft=(%.6f %.6f %.6f)",
    hombro.c_str(),
    out->t_sh.x, out->t_sh.y, out->t_sh.z,
    out->t_el.x, out->t_el.y, out->t_el.z,
    out->t_ft.x, out->t_ft.y, out->t_ft.z);

  return true;
}

bool JointPubNode::initLegOffsetsFromTF(tf2_ros::Buffer& tf_buffer,
                                        rclcpp::Logger logger,
                                        rclcpp::Clock::SharedPtr clock)
{
  struct Frames { std::string hombro, brazo, antebrazo, foot; };

  static const std::array<Frames, 4> frames = {{
    {"FR_Hombro", "FR_Brazo", "FR_Antebrazo", "FR_Foot"},
    {"RR_Hombro", "RR_Brazo", "RR_Antebrazo", "RR_Foot"},
    {"RL_Hombro", "RL_Brazo", "RL_Antebrazo", "RL_Foot"},
    {"FL_Hombro", "FL_Brazo", "FL_Antebrazo", "FL_Foot"}
  }};

  for (int i = 0; i < 4; ++i) {
    if (!tf_buffer.canTransform(frames[i].hombro, frames[i].brazo, tf2::TimePointZero) ||
        !tf_buffer.canTransform(frames[i].brazo, frames[i].antebrazo, tf2::TimePointZero) ||
        !tf_buffer.canTransform(frames[i].antebrazo, frames[i].foot, tf2::TimePointZero))
    {
      RCLCPP_INFO_THROTTLE(logger, *clock, 1000, "Waiting TF for leg %d offsets...", i);
      return false;
    }
  }

  for (int i = 0; i < 4; ++i) {
    LegOffsets tmp;
    if (!readLegOffsetsFromTF(tf_buffer,
          frames[i].hombro, frames[i].brazo, frames[i].antebrazo, frames[i].foot,
          &tmp, logger, clock))
    {
      return false;
    }
    body_->legs_[i]->leg_offsets_[i] = tmp;
  }

  RCLCPP_INFO(logger, "Leg offsets initialized from TF for all legs.");
  return true;
}

void JointPubNode::initTF()
{
  if (body_initialized_) return;

  static const std::array<std::string, 4> foot_frames = {{
    "FR_Foot", "RR_Foot", "RL_Foot", "FL_Foot"
  }};

  static const std::array<std::string, 4> hip_frames = {{
    "FR_HipFrame", "RR_HipFrame", "RL_HipFrame", "FL_HipFrame"
  }};

  for (int i = 0; i < 4; ++i) {
    if (!tf_buffer_.canTransform("base_link", foot_frames[i], tf2::TimePointZero)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Waiting for TF base_link -> %s", foot_frames[i].c_str());
      return;
    }
    if (!tf_buffer_.canTransform("base_link", hip_frames[i], tf2::TimePointZero)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Waiting for TF base_link -> %s", hip_frames[i].c_str());
      return;
    }
  }

  for (int i = 0; i < 4; ++i) {
    auto Th = tf_buffer_.lookupTransform("base_link", hip_frames[i], tf2::TimePointZero);
    body_->hips[i].x = static_cast<float>(Th.transform.translation.x);
    body_->hips[i].y = static_cast<float>(Th.transform.translation.y);
    body_->hips[i].z = static_cast<float>(Th.transform.translation.z);

    RCLCPP_INFO(get_logger(),
      "%s hip = (%.6f, %.6f, %.6f)",
      hip_frames[i].c_str(),
      body_->hips[i].x,
      body_->hips[i].y,
      body_->hips[i].z);
  }

  for (int i = 0; i < 4; ++i) {
    auto T = tf_buffer_.lookupTransform("base_link", foot_frames[i], tf2::TimePointZero);

    body_->legs_[i]->foot_home_x = body_->legs_[i]->foot_meas_x = body_->legs_[i]->foot_cmd_x =
      static_cast<float>(T.transform.translation.x);
    body_->legs_[i]->foot_home_y = body_->legs_[i]->foot_meas_y = body_->legs_[i]->foot_cmd_y =
      static_cast<float>(T.transform.translation.y);
    body_->legs_[i]->foot_home_z = body_->legs_[i]->foot_meas_z = body_->legs_[i]->foot_cmd_z =
      static_cast<float>(T.transform.translation.z);

    RCLCPP_INFO(get_logger(),
      "%s init foot = (%.3f, %.3f, %.3f)",
      foot_frames[i].c_str(),
      body_->legs_[i]->foot_meas_x,
      body_->legs_[i]->foot_meas_y,
      body_->legs_[i]->foot_meas_z);
  }

  if (!offsets_initialized_) {
    offsets_initialized_ = initLegOffsetsFromTF(tf_buffer_, get_logger(), get_clock());
    if (!offsets_initialized_) return;
  }

  body_initialized_ = true;
  init_timer_->cancel();
  RCLCPP_INFO(get_logger(), "Feet & Hips initialized from TF ");
}

bool JointPubNode::readFootPos(const std::string& foot_frame, float& x, float& y, float& z)
{
  try {
    auto T = tf_buffer_.lookupTransform("base_link", foot_frame, tf2::TimePointZero);
    x = static_cast<float>(T.transform.translation.x);
    y = static_cast<float>(T.transform.translation.y);
    z = static_cast<float>(T.transform.translation.z);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
      "TF lookup failed base_link -> %s : %s", foot_frame.c_str(), ex.what());
    return false;
  }
}

void JointPubNode::updateFeetFromTF()
{
  static const std::array<std::string, 4> foot_frames = {{
    "FR_Foot", "RR_Foot", "RL_Foot", "FL_Foot"
  }};

  for (int i = 0; i < 4; ++i) {
    float x, y, z;
    if (readFootPos(foot_frames[i], x, y, z)) {
      body_->legs_[i]->foot_meas_x = x;
      body_->legs_[i]->foot_meas_y = y;
      body_->legs_[i]->foot_meas_z = z;
    }
  }
}

// ======================= SOCKET / LOOP =======================
// (Aquí pegamos tal cual tu código original, sin cambios)

void JointPubNode::processSocket() {
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

    switch (code) {

      case 1: { // XY
        if (!walk_ON) {
          goalX = parser_->pX;
          goalY = parser_->pY;
          gait_.setGoalBase(goalX, goalY);
          RCLCPP_INFO(this->get_logger(), "Destino (%.3f, %.3f).", goalX, goalY);
        }
        break;
      }

      case 3: { // WL
        RCLCPP_INFO(this->get_logger(), "Walk command");
        walk_ON = parser_->walkMode;

        for (auto& leg : body_->legs_) {
          if (leg) leg->walk = true;
        }

        if (walk_ON) {
          if (gait_.isPaused) gait_.resume();
        } else {
          gait_.pause();
          body_->centerPos();
          for (auto& leg : body_->legs_) {
            if (leg) leg->walk = false;
          }
        }
        break;
      }

      case 4: { // OF
        const int leg_id = parser_->leg_id;
        RCLCPP_INFO(this->get_logger(),
                    "[OF] set_joint_offset() no existe en Leg (leg=%d, joint=%d, off=%d)",
                    leg_id, parser_->joint % 3, parser_->joint_offset);
        newPose = true;
        break;
      }

      case 5: { // AG
        const int leg_id = parser_->leg_id;
        RCLCPP_INFO(this->get_logger(),
                    "[AG] setServoAngle() no existe en Leg (leg=%d, joint=%d, angle=%d)",
                    leg_id, parser_->joint % 3, parser_->joint_angle);
        break;
      }

      case 6: { // LG
        int leg_id = parser_->leg_id;
        RCLCPP_INFO(this->get_logger(), "[LG] Shrinking leg id = %d\n", leg_id);

        body_->legs_[leg_id]->foot_cmd_x = body_->legs_[leg_id]->foot_home_x;
        body_->legs_[leg_id]->foot_cmd_y = body_->legs_[leg_id]->foot_home_y;
        body_->legs_[leg_id]->foot_cmd_z = 0.02f;

        newPose = true;
        break;
      }

      case 7: { // MV
        moveBody = true;
        break;
      }

      case 8: { // PI
        pitch_cmd = static_cast<float>(parser_->pitch / 180.0 * M_PI);
        horiz_control.resetPID();
        newPose = true;
        break;
      }

      case 9: { // RO
        roll_cmd = static_cast<float>(parser_->roll / 180.0 * M_PI);
        horiz_control.resetPID();
        newPose = true;
        break;
      }

      case 11: { // TL
        telem_ON_ = parser_->telem;
        break;
      }

      case 12: { // HZ
        horiz_control.hold_ON = parser_->hold_orient;
        horiz_control.resetControllers();
        newPose = true;
        break;
      }

      case 13: { // CG
        horiz_control.cg_control = parser_->cg_control;
        newPose = true;
        break;
      }

      case 14: { // FP
        const int leg_id = parser_->leg_id;
        RCLCPP_INFO(this->get_logger(),
                    "[FP] setFootPos() no encontrado en Leg (leg=%d, %f,%f,%f)",
                    leg_id, parser_->pX, parser_->pY, parser_->pZ);
        newPose = true;
        break;
      }

      case 16: { // VE
        break;
      }

      case 17: { // TR
        trot_gait   = parser_->trot_gait;
        trot_start  = this->now();
        trot_t_last_s = 0.0f;
        t_sim = 0.0f;
        RCLCPP_INFO(this->get_logger(), "Trot command");
        break;
      }

      case 18: { // OZ
        oscZ_amp = parser_->osc_amp / 100.0f;
        RCLCPP_INFO(this->get_logger(), "OZ command: %f", oscZ_amp);
        break;
      }

      case 19: { // OW
        osc_w = parser_->osc_w;
        RCLCPP_INFO(this->get_logger(), "OW command: %f", osc_w);
        break;
      }

      case 20: { // OP
        osc_phi = parser_->osc_phi / 100.0f;
        RCLCPP_INFO(this->get_logger(), "OP command: %f", osc_phi);
        break;
      }

      case 23: { // OX
        oscX_amp = parser_->oscX_amp / 100.0f;
        RCLCPP_INFO(this->get_logger(), "OX command: %f", oscX_amp);
        break;
      }

      case 24: { // OY
        oscY_amp = parser_->oscY_amp / 100.0f;
        RCLCPP_INFO(this->get_logger(), "OY command: %f", oscY_amp);
        break;
      }

      case -1: { // R
        gait_.pause();
        walk_ON = false;
        trot_gait = false;
        roll_cmd = 0.0f;
        pitch_cmd = 0.0f;
        horiz_control.hold_ON = false;
        centerBody = true;
        RCLCPP_INFO(this->get_logger(), "[R] Reset");
        break;
      }

      default: {
        RCLCPP_WARN(this->get_logger(),
                    "Comando desconocido: #%s; (code=%d)",
                    label.c_str(), code);
        break;
      }
    }
  }
}

void JointPubNode::tick() {
  if (!started_) {
    if (now() >= start_time_) {
      started_ = true;
      RCLCPP_INFO(this->get_logger(), "¡Comenzando a andar!");
    }
  }
  else if (body_initialized_) {
    body_->worldX = static_cast<float>(odom_x_.load());
    body_->worldY = static_cast<float>(odom_y_.load());
    body_->worldZ = static_cast<float>(odom_z_.load());
    roll_meas  = static_cast<float>(imu_roll_.load());
    pitch_meas = static_cast<float>(imu_pitch_.load());
    yaw_meas   = static_cast<float>(odom_yaw_.load());

    updateFeetFromTF();

    if (!isMoving && !trot_gait){
      float x, y, z;
      body_->getMeanFootPos(&x, &y, &z);
      height = -z;
    }

    horiz_control.newRobotOrient(
      body_->roll,
      body_->pitch,
      height,
      roll_meas,
      pitch_meas
    );

    body_->cg_x = horiz_control.cg_x;
    body_->cg_y = horiz_control.cg_y;

    static float last_cgx = 0.0f;
    static float last_cgy = 0.0f;

    if (horiz_control.cg_control) {
      const float cx =  (horiz_control.cg_x - last_cgx);
      const float cy = -(horiz_control.cg_y - last_cgy);

      last_cgx = horiz_control.cg_x;
      last_cgy = horiz_control.cg_y;

      body_->move(cx, cy, 0.0f);
      newPose = true;
    }

    if (horiz_control.hold_ON) {
      horiz_control.stabilize(
        roll_cmd, pitch_cmd, roll_meas, pitch_meas, &roll, &pitch
      );
    } else {
      roll = roll_cmd;
      pitch = pitch_cmd;
    }

    body_->roll  = roll;
    body_->pitch = pitch;
    body_->yaw   = yaw_meas;

    if (newPose){
      body_->updatePose(true);
      newPose =  false;
    }

    if (walk_ON) {
      float haveTarget = false;
      if (goalX != 0.0f || goalY != 0.0f) haveTarget = true;
      gait_.walk(*this->get_clock(), get_logger(), walk_ON, newPose, haveTarget, goalX, goalY);

      if (gait_.isPaused && body_->isIdle) centerBody = true;
      else isMoving = true;

      newPose = true;
    }

    if (moveBody) {
      float food_speed = 1.0f;
      float cmd_x = body_->legs_[FR]->foot_cmd_x;
      float cmd_y = body_->legs_[FR]->foot_cmd_y;
      float cmd_z = body_->legs_[FR]->foot_cmd_z;
      RCLCPP_INFO(get_logger(), "Foot cmd (%.3f, %.3f, %.3f)", cmd_x, cmd_y, cmd_z);

      body_->legs_[FR]->footPlanner.setPlanner(get_logger(), food_speed, cmd_x, cmd_y, cmd_z,
                                               cmd_x + 0.02f, cmd_y + 0.0f, cmd_z + 0.03f, 0.03f);
      newPose = true;
      if(body_->legs_[FR]->arrived2pos()) moveBody =  false;
    }

    if (centerBody) {
      for (auto &leg : body_->legs_) {
        if (!leg) continue;
        leg->foot_cmd_x = leg->foot_home_x;
        leg->foot_cmd_y = leg->foot_home_y;
        leg->foot_cmd_z = leg->foot_home_z;
      }
      newPose = true;
      centerBody = false;
    }

    if (trot_gait) {
      if (!body_initialized_) return;

      float t = static_cast<float>((this->now() - trot_start).seconds());
      float dt = t - trot_t_last_s;
      trot_t_last_s  = t;
      t_sim += dt * osc_w;

      dz = oscZ_amp * std::sin(t_sim);
      dz = (dz < 0) ? 0 : dz;
      body_->legs_[FR]->foot_cmd_z = body_->legs_[FR]->foot_meas_z + dz;
      body_->legs_[RL]->foot_cmd_z = body_->legs_[RL]->foot_meas_z + dz;

      dx = oscX_amp * std::sin(t_sim - static_cast<float>(M_PI/2) + oscX_phi);
      body_->legs_[FR]->foot_cmd_x = body_->legs_[FR]->foot_meas_x + dx;
      body_->legs_[RL]->foot_cmd_x = body_->legs_[RL]->foot_meas_x + dx;

      dy = oscY_amp * std::sin(t_sim - static_cast<float>(M_PI/2) + oscY_phi);
      body_->legs_[FR]->foot_cmd_y = body_->legs_[FR]->foot_meas_y + dy;
      body_->legs_[RL]->foot_cmd_y = body_->legs_[RL]->foot_meas_y + dy;

      dz = oscZ_amp * std::sin(t_sim + osc_phi);
      dz = (dz < 0) ? 0 : dz;
      body_->legs_[FL]->foot_cmd_z = body_->legs_[FL]->foot_meas_z + dz;
      body_->legs_[RR]->foot_cmd_z = body_->legs_[RR]->foot_meas_z + dz;

      dx = oscX_amp * std::sin(t_sim + osc_phi - static_cast<float>(M_PI/2) + oscX_phi);
      body_->legs_[FL]->foot_cmd_x = body_->legs_[FL]->foot_meas_x + dx;
      body_->legs_[RR]->foot_cmd_x = body_->legs_[RR]->foot_meas_x + dx;

      dy = oscY_amp * std::sin(t_sim + osc_phi - static_cast<float>(M_PI/2) + oscX_phi);
      body_->legs_[FL]->foot_cmd_y = body_->legs_[FL]->foot_meas_y + dy;
      body_->legs_[RR]->foot_cmd_y = body_->legs_[RR]->foot_meas_y + dy;

      if(t >= 20.0f && csv_file_.is_open()) csv_file_.close();
      newPose = true;
    }

    if(!trot_gait && !centerBody && !walk_ON) {
      centerBody = true;
    }

    now_ms = this->get_clock()->now().nanoseconds() / 1'000'000;

    if ((now_ms - last_telem_ms_) >= telem_rate_ms_ && telem_ON_) {
      last_telem_ms_ = now_ms;
      (void)sendTelemetryFrames();
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
      for (int j = 0; j < 3; ++j) positions.push_back(pos[j]);
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
    traj_pub_->publish(jt);
  }
}

// ======================= TELEMETRY / SOCKET =======================

bool JointPubNode::sendAllNonBlocking(int fd, const char* data, size_t len, rclcpp::Logger logger)
{
  size_t sent = 0;
  while (sent < len) {
    ssize_t n = ::send(fd, data + sent, len - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      continue;
    }
    if (n == 0) {
      RCLCPP_WARN(logger, "send(): conexión cerrada por el peer");
      return false;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
    if (errno == EINTR) continue;
    RCLCPP_ERROR(logger, "send(): %s", strerror(errno));
    return false;
  }
  return true;
}

inline void JointPubNode::appendNumber(std::ostringstream& oss, double v)
{
  oss.setf(std::ios::fixed);
  oss.precision(6);
  oss << v;
}

bool JointPubNode::sendTelemetryFrames()
{
  if (conn_fd_ < 0) return false;

  std::ostringstream a;
  a << '@';
  appendNumber(a, body_->roll); a << ',';
  appendNumber(a, body_->pitch); a << ',';
  appendNumber(a, body_->yaw); a << ',';
  appendNumber(a, body_->worldX); a << ',';
  appendNumber(a, body_->worldY); a << ',';
  appendNumber(a, body_->worldZ); a << ',';
  appendNumber(a, horiz_control.cg_x); a << ',';
  appendNumber(a, horiz_control.cg_y); a << ',';
  a << ";\n";

  const std::string a_str = a.str();
  if (!sendAllNonBlocking(conn_fd_, a_str.c_str(), a_str.size(), get_logger())) return false;

  std::ostringstream b;
  b << '&';
  b << body_->posX << ',' << body_->posY << ',' << body_->posZ << ',';
  b << body_->legs_[FR]->foot_meas_x << ',' << body_->legs_[FR]->foot_meas_y << ',' << body_->legs_[FR]->foot_meas_z << ',';
  b << body_->legs_[RR]->foot_meas_x << ',' << body_->legs_[RR]->foot_meas_y << ',' << body_->legs_[RR]->foot_meas_z << ',';
  b << body_->legs_[FL]->foot_meas_x << ',' << body_->legs_[FL]->foot_meas_y << ',' << body_->legs_[FL]->foot_meas_z << ',';
  b << body_->legs_[RL]->foot_meas_x << ',' << body_->legs_[RL]->foot_meas_y << ',' << body_->legs_[RL]->foot_meas_z;
  b << ";\n";

  const std::string b_str = b.str();
  if (!sendAllNonBlocking(conn_fd_, b_str.c_str(), b_str.size(), get_logger())) return false;

  std::ostringstream c;
  c << '%';
  c << body_->legs_[FR]->ang1 << ',' << body_->legs_[FR]->ang2 << ',' << body_->legs_[FR]->ang3 << ',';
  c << body_->legs_[RR]->ang1 << ',' << body_->legs_[RR]->ang2 << ',' << body_->legs_[RR]->ang3 << ',';
  c << body_->legs_[FL]->ang1 << ',' << body_->legs_[FL]->ang2 << ',' << body_->legs_[FL]->ang3 << ',';
  c << body_->legs_[RL]->ang1 << ',' << body_->legs_[RL]->ang2 << ',' << body_->legs_[RL]->ang3;
  c << ";\n";

  const std::string c_str = c.str();
  if (!sendAllNonBlocking(conn_fd_, c_str.c_str(), c_str.size(), get_logger())) return false;

  return true;
}

void JointPubNode::openSocketAndListen()
{
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
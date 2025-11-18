// main.cpp  (Nodo gait_duty_autoadapt + socket + marcha + trote + logger + IK)
//
// - Marcha "duty" (ir a X) y trote usan el MISMO CPG de fase,
//   pero se genera una trayectoria cartesiana del pie por pata
//   y se pasa por la IK de quadbot::LegKinematics.
// - TR:1 = trote libre (mismas fases, otras frecuencias/amplitudes).
// - TR:0 = paro total, postura neutra.
// - XY:... = manda meta en X y activa la marcha “duty” de ir a X.
// - Log de joints en:
//      /home/lucia/Desktop/gait_joints_log.txt
//
// Requiere: keyboard_cmd_parser.hpp y kinematics.hpp en include path.

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <unordered_map>
#include <vector>
#include <string>
#include <iostream>
#include <limits>
#include <iomanip>
#include <cstdint>
#include <fstream>

// Sockets
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

// Tu parser
#include "keyboard_cmd_parser.hpp"

// IK de patas
#include "kinematics.hpp"

using std::placeholders::_1;

// ======================= CONFIG FIJA (de la CLI original) =======================
static const char* CONTROLLER_NAME = "joint_trajectory_controller";

// Marcha “duty” (caminar estable para ir a X)
static constexpr double FREQUENCY_HZ      = 0.55;
static constexpr double DUTY_STANCE       = 0.60;
static constexpr double PUBLISH_RATE_HZ   = 100.0;
static constexpr double POINT_HORIZON_SEC = 0.14;

static constexpr double DRIVER_AMPL       = 0.32;
static constexpr double AUX_AMPL          = 0.12;   // ya no se usa en IK, se deja por compat
static constexpr double ELBOW_LIFT        = 0.24;
static constexpr double BIAS_SHOULDER     = +0.08;
static constexpr double BIAS_HIP_ABD      = 0.06;

static constexpr bool   GEOM_COMP         = true;
static constexpr bool   DO_PRINT          = true;
static constexpr bool   PRINT_ACTUAL      = false;
static constexpr double PRINT_RATE_HZ     = 1.0;

// Límites
static constexpr double JOINT_LOWER       = -1.57;
static constexpr double JOINT_UPPER       = +1.57;
static constexpr double SOFT_MARGIN       = 0.12;

// Odometría
static constexpr double VX_WINDOW_SEC     = 2.0;

// “Ir a X”
static constexpr double GOAL_TOLERANCE_DEFAULT = 0.02; // metros

// ======================= PARÁMETROS ESPECÍFICOS DE TROTE =======================
static constexpr double TROT_FREQ_HZ       = 2.2;
static constexpr double TROT_DUTY_STANCE   = 0.42;
static constexpr double TROT_DRIVER_AMPL   = 0.55;
static constexpr double TROT_ELBOW_LIFT    = 0.48;
static constexpr double TROT_SHOULDER_GAIN = 1.7;  // no se usa en IK
static constexpr double TROT_HIP_FOLLOW    = 0.55; // no se usa en IK

// Logger
static constexpr const char* JOINT_LOG_PATH = "/home/lucia/Desktop/gait_joints_log.txt";

// ======================= GEOMETRÍA DE PATA / GAIT =======================
// Ajusta estos parámetros a tu robot real

// Longitudes de la pata para IK (m) — deben coincidir con LegKinematics
static constexpr double L0 = 0.04;  // offset lateral
static constexpr double L1 = 0.09;  // primer eslabón
static constexpr double L2 = 0.12;  // segundo eslabón

// Posiciones neutras del pie en marco de hombro de cada pata
// (x hacia delante, y hacia afuera, z hacia abajo negativo)
static constexpr double BASE_X_FRONT = 0.05;   // pies delanteros adelantados
static constexpr double BASE_X_REAR  = -0.03;  // pies traseros algo retrasados
static constexpr double Y_OFFSET     = 0.03;   // distancia lateral
static constexpr double FOOT_Z_NEUTRAL = -0.14; // altura neutra del pie

// Zancada/lift base (muy conservador)
static constexpr double STEP_LENGTH_BASE  = 0.03; // 3 cm
static constexpr double SWING_HEIGHT_BASE = 0.02; // 2 cm

// Direcciones/gain por pata (para marcha/trote)
struct Gains {
  double dir      = 1.0;
  double shoulder = 1.0;
  double elbow    = 2.5;
};

struct LegDef {
  std::string name;
  int elbow, shoulder, hip; // índices
  double phase;             // fase base
  Gains gains;
  double lift_scale;        // factor extra de lift por pata
};

class DutyGaitAuto : public rclcpp::Node {
public:
  DutyGaitAuto()
  : Node("gait_duty_autoadapt"),
    ik_(L0, L1, L2),
    step_length_(STEP_LENGTH_BASE),
    swing_height_(SWING_HEIGHT_BASE),
    print_period_(1.0 / std::max(1e-6, PRINT_RATE_HZ)),
    last_print_t_(this->now().seconds()),
    printed_header_(false),
    has_goal_(false),
    goal_x_(0.0),
    goal_tolerance_(GOAL_TOLERANCE_DEFAULT),
    move_dir_(0.0),
    last_x_(std::numeric_limits<double>::quiet_NaN()),
    run_gait_(false),
    have_odom_(false),
    trot_gait_(false),
    sockfd_(-1),
    client_fd_(-1),
    t0_(rclcpp::Clock().now().seconds())
  {
    // ---------- Parámetro del tópico de odometría ----------
    std::string odom_topic = this->declare_parameter<std::string>(
      "odom_topic", "/odom");

    // ---------- Puerto TCP para comandos ----------
    auto tcp_port_param = this->declare_parameter<int64_t>("tcp_port", 9000);
    int tcp_port = static_cast<int>(tcp_port_param);
    setupSocketServer(tcp_port);

    // Joints
    joints_.reserve(12);
    for (int i = 1; i <= 12; ++i)
      joints_.push_back("Revolution_" + std::to_string(i));

    // Patas y fases: FL/RR en fase; FR/RL a π → trote diagonal.
    legs_ = {
      {"FL", 0, 1, 2, 0.0,  Gains{+1.0, 0.95, 2.5}, 1.40},
      {"RL", 3, 4, 5, M_PI, Gains{+1.0, 0.95, 3.0}, 1.40},
      {"RR", 6, 7, 8, 0.0,  Gains{+1.0, 1.05, 2.5}, 1.80},
      {"FR", 9,10,11, M_PI, Gains{+1.0, 1.05, 2.5}, 1.30},
    };

    // Publisher
    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/" + std::string(CONTROLLER_NAME) + "/joint_trajectory", 10);

    // QoS BEST_EFFORT para odom/js
    auto qos_odom = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 20))
                      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    auto qos_js   = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 50))
                      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Subs
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos_odom, std::bind(&DutyGaitAuto::odomCb, this, _1));
    RCLCPP_INFO(this->get_logger(), "Suscrito a odometría: %s", odom_topic.c_str());

    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", qos_js, std::bind(&DutyGaitAuto::jsCb, this, _1));

    // Meta de posición X
    goal_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/goal_x", 1, std::bind(&DutyGaitAuto::goalCb, this, _1));

    // Interruptor opcional de marcha
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/enable_gait", 1, std::bind(&DutyGaitAuto::enableCb, this, _1));

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1e-6, PUBLISH_RATE_HZ)),
      std::bind(&DutyGaitAuto::tick, this));

    // Logger
    log_file_.open(JOINT_LOG_PATH, std::ios::out | std::ios::trunc);
    if (!log_file_) {
      RCLCPP_ERROR(this->get_logger(),
                   "No se pudo abrir el log de joints en %s", JOINT_LOG_PATH);
    } else {
      log_file_ << "# t";
      for (const auto &jn : joints_) {
        log_file_ << " " << jn;
      }
      log_file_ << "\n";
      log_file_.flush();
      RCLCPP_INFO(this->get_logger(),
                  "Logging de joints en: %s", JOINT_LOG_PATH);
    }

    RCLCPP_INFO(this->get_logger(),
      "Gait duty autoadapt (CPG + IK): freq=%.3f Hz, duty=%.3f, geom_comp=%s | starts STOPPED",
      FREQUENCY_HZ, DUTY_STANCE, GEOM_COMP ? "true" : "false");
  }

  ~DutyGaitAuto() override {
    if (client_fd_ >= 0) ::close(client_fd_);
    if (sockfd_ >= 0) ::close(sockfd_);
    if (log_file_.is_open()) log_file_.close();
  }

private:
  // ---------- helpers ----------
  static double clamp01(double x) {
    if (x < 0.0) return 0.0;
    if (x > 1.0) return 1.0;
    return x;
  }

  static double half_cosine(double x) {
    x = clamp01(x);
    return 0.5 - 0.5 * std::cos(M_PI * x);
  }

  static double triangle(double x) {
    x = x - std::floor(x);
    if (x < 0.25)      return 4.0 * x;
    else if (x < 0.75) return 2.0 - 4.0 * x;
    else               return -4.0 + 4.0 * x;
  }

  static void duty_profile(double phase, double duty,
                           double &s_out, bool &in_stance_out, double &tau_out)
  {
    const double twopi = 2.0 * M_PI;
    double ph = std::fmod(phase, twopi);
    if (ph < 0) ph += twopi;
    const double u = ph / twopi;

    if (u < duty) {
      double tau = u / duty;
      double s   = -1.0 + tau * 1.0;   // stance: -1 → 0
      s_out = s;
      in_stance_out = true;
      tau_out = tau;
    } else {
      double tau = (u - duty) / std::max(1e-6, (1.0 - duty));
      double s   = 0.0 + tau * 1.0;    // swing: 0 → +1
      s_out = s;
      in_stance_out = false;
      tau_out = tau;
    }
  }

  static double soft_limit(double cmd) {
    const double up = JOINT_UPPER - SOFT_MARGIN;
    const double lo = JOINT_LOWER + SOFT_MARGIN;
    if (cmd > up) return up - 0.02;
    if (cmd < lo) return lo + 0.02;
    return cmd;
  }

  // ---------- callbacks ----------
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const double t = this->now().seconds();
    const double vx = msg->twist.twist.linear.x;
    vx_hist_.emplace_back(t, vx);
    const double tmin = t - VX_WINDOW_SEC;
    while (!vx_hist_.empty() && vx_hist_.front().first < tmin) {
      vx_hist_.pop_front();
    }
    last_x_ = msg->pose.pose.position.x;
    have_odom_ = true;
  }

  void jsCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      last_positions_[msg->name[i]] = msg->position[i];
    }
  }

  void enableCb(const std_msgs::msg::Bool::SharedPtr msg) {
    run_gait_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "run_gait_ (marcha) = %s",
                run_gait_ ? "true" : "false");
    if (!run_gait_ && !trot_gait_) {
      auto neutral = neutral_posture();
      publish_(neutral);
      logJoints(this->now().seconds(), neutral);
    }
  }

  void goalCb(const std_msgs::msg::Float64::SharedPtr msg) {
    if (trot_gait_) {
      RCLCPP_WARN(this->get_logger(),
                  "Meta recibida pero estoy en trote. Ignorando goal_x=%.3f", msg->data);
      return;
    }

    if (has_goal_) {
      RCLCPP_WARN(this->get_logger(),
                  "Ya hay meta activa (%.3f). Ignoro nueva meta: %.3f",
                  goal_x_, msg->data);
      return;
    }
    goal_x_ = msg->data;
    has_goal_ = true;

    if (!have_odom_) {
      run_gait_ = false;
      move_dir_ = 0.0;
      RCLCPP_WARN(this->get_logger(),
        "Recibí meta X=%.3f pero aún no hay odometría. Quedo parado hasta recibir /odom.",
        goal_x_);
      return;
    }

    run_gait_ = true;
    const double err = goal_x_ - last_x_;
    move_dir_ = (err >= 0.0) ? +1.0 : -1.0;
    for (auto & L : legs_) L.gains.dir = move_dir_;
    RCLCPP_INFO(this->get_logger(),
      "Nueva meta X=%.3f m | x=%.3f | dir=%+g | tol=%.3f",
      goal_x_, last_x_, move_dir_, goal_tolerance_);
  }

  double avg_vx() const {
    if (vx_hist_.empty()) return std::numeric_limits<double>::quiet_NaN();
    double s = 0.0;
    for (const auto& p : vx_hist_) s += p.second;
    return s / static_cast<double>(vx_hist_.size());
  }

  // ============================================================
  // Socket + CommandParser
  // ============================================================
  void setupSocketServer(int port) {
    sockfd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "socket() fallo: %s", std::strerror(errno));
      return;
    }

    int opt = 1;
    ::setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));

    if (::bind(sockfd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "bind() fallo: %s", std::strerror(errno));
      ::close(sockfd_); sockfd_ = -1;
      return;
    }

    if (::listen(sockfd_, 1) < 0) {
      RCLCPP_ERROR(this->get_logger(), "listen() fallo: %s", std::strerror(errno));
      ::close(sockfd_); sockfd_ = -1;
      return;
    }

    int flags = fcntl(sockfd_, F_GETFL, 0);
    fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

    RCLCPP_INFO(this->get_logger(), "Servidor TCP escuchando en puerto %d", port);
  }

  void acceptClientIfNeeded() {
    if (sockfd_ < 0) return;
    if (client_fd_ >= 0) return;

    sockaddr_in caddr{};
    socklen_t clen = sizeof(caddr);
    int fd = ::accept(sockfd_, reinterpret_cast<sockaddr*>(&caddr), &clen);
    if (fd < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "accept() fallo: %s", std::strerror(errno));
      }
      return;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    client_fd_ = fd;
    cmd_parser_ = std::make_unique<CommandParser>(client_fd_, this->get_logger());
    RCLCPP_INFO(this->get_logger(), "Cliente TCP conectado");
  }

  void processSocketCommands() {
    acceptClientIfNeeded();
    if (!cmd_parser_) return;

    while (cmd_parser_->readNext()) {
      int cmd = cmd_parser_->last_code();
      if (cmd == 0) continue;

      switch (cmd) {
        case 1: { // XY: pX,pY,pZ -> meta en X
          double px = cmd_parser_->pX;  // mm
          double goal_m = -px / 1000.0; // como en Arduino (robotX = -pX)
          auto msg = std_msgs::msg::Float64();
          msg.data = goal_m;
          goalCb(std::make_shared<std_msgs::msg::Float64>(msg));
          break;
        }
        case 2: { // SR: servos ON/OFF -> enable_gait
          bool on = (cmd_parser_->servoMode == 1);
          auto msg = std_msgs::msg::Bool();
          msg.data = on;
          enableCb(std::make_shared<std_msgs::msg::Bool>(msg));
          break;
        }
        case 3:
          // WL: walk mode (no se usa aquí, la marcha se controla por goal_x)
          break;
        case 16:
          // VE: velX, velY -> no usado
          break;
        case 17: { // TR: trot_gait ON/OFF
          bool new_trot = (cmd_parser_->trot_gait != 0);
          if (new_trot == trot_gait_) break;

          trot_gait_ = new_trot;
          if (trot_gait_) {
            has_goal_ = false;
            run_gait_ = true;
            move_dir_ = +1.0;
            for (auto & L : legs_) L.gains.dir = move_dir_;
            RCLCPP_INFO(this->get_logger(), "Trote libre ON (TR:1)");
          } else {
            trot_gait_ = false;
            run_gait_  = false;
            has_goal_  = false;
            move_dir_  = 0.0;
            RCLCPP_INFO(this->get_logger(),
                        "Trote libre OFF (TR:0), paro y postura neutra");
            auto neutral = neutral_posture();
            publish_(neutral);
            logJoints(this->now().seconds(), neutral);
          }
          break;
        }
        default:
          break;
      }
    }
  }

  // ---------- postura neutra ----------
  std::vector<double> neutral_posture() {
    std::vector<double> q(joints_.size(), 0.0);
    for (const auto& L : legs_) {
      double q_sh = soft_limit(BIAS_SHOULDER * L.gains.shoulder);
      double q_hp = soft_limit(((L.name == "FL" || L.name == "FR")
                                ? +BIAS_HIP_ABD : -BIAS_HIP_ABD));
      double q_el = soft_limit(0.06); // un poco flexionada
      q[L.shoulder] = q_sh;
      q[L.hip]      = q_hp;
      q[L.elbow]    = q_el;
    }
    return q;
  }

  // ============================================================
  // Trayectoria del pie por pata en marco de su hombro
  // (x adelante, y hacia afuera, z hacia abajo negativo)
  // ============================================================
  void foot_trajectory_(const LegDef &L,
                        double tau,
                        bool in_stance,
                        double dir,
                        double &x, double &y, double &z) const
  {
    // Posición base neutral en X
    if (L.name == "FL" || L.name == "FR") {
      x = BASE_X_FRONT;
    } else {
      x = BASE_X_REAR;
    }

    // Posición base neutral en Y (izq/dcha)
    if (L.name == "FL" || L.name == "RL") {
      y = +Y_OFFSET;  // patas izquierdas
    } else {
      y = -Y_OFFSET;  // patas derechas
    }

    // Altura neutra
    z = FOOT_Z_NEUTRAL;

    const double Ls = step_length_;
    const double Hs = swing_height_ * L.lift_scale;

    double dx = 0.0;
    double dz = 0.0;

    if (in_stance) {
      // Pie en contacto, el cuerpo avanza → el pie se "mueve hacia atrás"
      dx = -dir * (tau - 0.5) * Ls;
      dz = 0.0;
    } else {
      // Swing: adelantar pie + lift medio coseno
      dx = +dir * (tau - 0.5) * Ls;
      dz = Hs * half_cosine(tau);
    }

    x += dx;
    z += dz;  // z sigue siendo negativa, dz > 0 lo acerca al cuerpo (sube)
  }

  // ============================================================
  // CPG + IK (marcha y trote)
  // ============================================================
  void generateDutyCPG(double freq_hz,
                       double duty,
                       double driver_ampl,
                       double elbow_lift,
                       double /*shoulder_scale*/,
                       double /*hip_follow_gain*/,
                       std::vector<double> &q_out)
  {
    const double now_s = this->now().seconds();
    const double t     = now_s - t0_;
    const double omega = 2.0 * M_PI * freq_hz;

    // Escalado suave de zancada/lift
    const double k_len  = 0.5 + std::fabs(driver_ampl);  // ~[0.5..1.1]
    const double k_lift = 0.5 + std::fabs(elbow_lift);

    step_length_  = STEP_LENGTH_BASE  * k_len;
    swing_height_ = SWING_HEIGHT_BASE * k_lift;

    q_out.assign(joints_.size(), 0.0);

    for (const auto& L : legs_) {
      const int elbow    = L.elbow;
      const int shoulder = L.shoulder;
      const int hip      = L.hip;

      const double phi = omega * t + L.phase;

      double s          = 0.0;
      bool   in_stance  = true;
      double tau        = 0.0;
      duty_profile(phi, duty, s, in_stance, tau);

      double dir = (L.gains.dir >= 0.0) ? +1.0 : -1.0;

      // Trayectoria del pie
      double x, y, z;
      foot_trajectory_(L, tau, in_stance, dir, x, y, z);

      // IK lado izq/dcha según tu implementación
      double q0, q1, q2;
      bool ok = false;
      if (L.name == "FL" || L.name == "RL") {
        ok = ik_.computeLeft(x, y, z, q0, q1, q2);
      } else {
        ok = ik_.computeRight(x, y, z, q0, q1, q2);
      }

      if (!ok) {
        // Si la IK falla, neutro seguro
        q_out[hip]      = soft_limit(0.0);
        q_out[shoulder] = soft_limit(0.0);
        q_out[elbow]    = soft_limit(0.06);
        continue;
      }

      // Mapeo según tu IK:
      //  q0 = abducción (hip),
      //  q1 = hombro (pitch),
      //  q2 = codo.
      double q_hip      = q0;
      double q_shoulder = q1;
      double q_elbow    = q2;

      // Offsets para parecerse a tu postura neutra del log
      const double hip_offset      = (L.name == "FL" || L.name == "FR")
                                     ? +BIAS_HIP_ABD : -BIAS_HIP_ABD;
      const double shoulder_offset = BIAS_SHOULDER * L.gains.shoulder;
      const double elbow_offset    = 0.06;

      q_hip      += hip_offset;
      q_shoulder += shoulder_offset;
      q_elbow    += elbow_offset;

      // Compensación geométrica opcional sobre el hombro
      if (GEOM_COMP) {
        double q_sh_soft = soft_limit(q_shoulder);
        if (std::fabs(q_sh_soft - q_shoulder) > 1e-6) {
          double delta = q_shoulder - q_sh_soft;
          q_shoulder   = q_sh_soft;
          q_elbow     += 0.6 * delta * L.gains.elbow;
        }
      }

      // clamps finales
      q_out[hip]      = soft_limit(q_hip);
      q_out[shoulder] = soft_limit(q_shoulder);
      q_out[elbow]    = soft_limit(q_elbow);
    }
  }

  // ============================================================
  // Logging a fichero
  // ============================================================
  void logJoints(double now_s, const std::vector<double> &q) {
    if (!log_file_.is_open()) return;
    double t_rel = now_s - t0_;
    log_file_ << std::fixed << std::setprecision(4) << t_rel;
    for (size_t i = 0; i < joints_.size(); ++i) {
      double v = (i < q.size()) ? q[i] : 0.0;
      log_file_ << " " << std::showpos << std::fixed << std::setprecision(6) << v
                << std::noshowpos;
    }
    log_file_ << "\n";
  }

  // ============================================================
  // tick principal
  // ============================================================
  void tick() {
    processSocketCommands();

    const double now_s = this->now().seconds();

    // ----- MODO TROTE LIBRE -----
    if (trot_gait_) {
      if (move_dir_ == 0.0) {
        move_dir_ = +1.0;
        for (auto & L : legs_) L.gains.dir = move_dir_;
      }

      std::vector<double> q;
      generateDutyCPG(
        TROT_FREQ_HZ,
        TROT_DUTY_STANCE,
        TROT_DRIVER_AMPL,
        TROT_ELBOW_LIFT,
        TROT_SHOULDER_GAIN,
        TROT_HIP_FOLLOW,
        q
      );

      publish_(q);
      logJoints(now_s, q);
      return;
    }

    // ----- MODO MARCHA “IR A X” -----

    if (has_goal_ && !have_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
        "Sin odometría (/odom_topic) → no puedo avanzar hacia la meta. Esperando...");
      auto neutral = neutral_posture();
      publish_(neutral);
      logJoints(now_s, neutral);
      return;
    }

    if (!run_gait_) {
      auto neutral = neutral_posture();
      publish_(neutral);
      logJoints(now_s, neutral);
      return;
    }

    if (has_goal_ && have_odom_) {
      double err = goal_x_ - last_x_;

      if (move_dir_ == 0.0) {
        move_dir_ = (err >= 0.0) ? +1.0 : -1.0;
        for (auto & L : legs_) L.gains.dir = move_dir_;
      } else {
        for (auto & L : legs_) L.gains.dir = (err >= 0.0) ? +1.0 : -1.0;
      }

      if (std::fabs(err) <= goal_tolerance_) {
        has_goal_ = false;
        run_gait_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "Objetivo alcanzado: x=%.3f ≈ %.3f (tol=%.3f). Parando.",
                    last_x_, goal_x_, goal_tolerance_);
        auto neutral = neutral_posture();
        publish_(neutral);
        logJoints(now_s, neutral);
        return;
      }

      if ((err > 0.0 && move_dir_ < 0.0) || (err < 0.0 && move_dir_ > 0.0)) {
        has_goal_ = false;
        run_gait_ = false;
        RCLCPP_WARN(this->get_logger(),
          "Meta rebasada: x=%.3f, goal=%.3f. Paro para no sobrepasar.",
          last_x_, goal_x_);
        auto neutral = neutral_posture();
        publish_(neutral);
        logJoints(now_s, neutral);
        return;
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
        "goal_x=%.3f  x=%.3f  err=%.3f  tol=%.3f  dir=%+.0f",
        goal_x_, last_x_, err, goal_tolerance_, move_dir_);
    }

    // Marcha duty normal (CPG + IK)
    std::vector<double> q;
    generateDutyCPG(
      FREQUENCY_HZ,
      DUTY_STANCE,
      DRIVER_AMPL,
      ELBOW_LIFT,
      1.0,
      0.35,
      q
    );

    publish_(q);
    logJoints(now_s, q);

    // impresión periódica
    if (DO_PRINT) {
      const double vx = avg_vx();
      if ((now_s - last_print_t_) >= print_period_) {
        last_print_t_ = now_s;
        if (!printed_header_) {
          std::cout << "# t[s]  joint_name       cmd(rad)";
          if (PRINT_ACTUAL) std::cout << "    act(rad)    err(rad)";
          std::cout << std::endl;
          printed_header_ = true;
        }
        for (size_t i = 0; i < joints_.size(); ++i) {
          const std::string& name = joints_[i];
          const double cmd = q[i];
          if (PRINT_ACTUAL) {
            const auto it = last_positions_.find(name);
            const double act = (it == last_positions_.end())
              ? std::numeric_limits<double>::quiet_NaN()
              : it->second;
            const double err = (std::isnan(act)
              ? std::numeric_limits<double>::quiet_NaN()
              : (act - cmd));
            std::cout << "t=" << std::fixed << std::setw(6) << std::setprecision(2)
                      << (now_s - t0_)
                      << "  " << std::left << std::setw(14) << name
                      << "  cmd=" << std::showpos << std::fixed << std::setprecision(3)
                      << cmd << std::noshowpos
                      << "   act=" << std::showpos << std::fixed << std::setprecision(3)
                      << act << std::noshowpos
                      << "   err=" << std::showpos << std::fixed << std::setprecision(3)
                      << err << std::noshowpos
                      << std::endl;
          } else {
            std::cout << "t=" << std::fixed << std::setw(6) << std::setprecision(2)
                      << (now_s - t0_)
                      << "  " << std::left << std::setw(14) << name
                      << "  cmd=" << std::showpos << std::fixed << std::setprecision(3)
                      << cmd << std::noshowpos
                      << std::endl;
          }
        }
        if (!std::isnan(vx)) {
          std::cout << "vx̄≈ " << std::showpos << std::fixed << std::setprecision(3)
                    << vx << " m/s" << std::noshowpos
                    << " | goal: " << (has_goal_ ? "activo" : "—")
                    << " | run_gait: " << (run_gait_ ? "ON" : "OFF")
                    << " | trot: " << (trot_gait_ ? "ON" : "OFF")
                    << std::endl;
        }
      }
    }
  }

  void publish_(const std::vector<double>& positions) {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joints_;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = positions;

    builtin_interfaces::msg::Duration d;
    d.sec = static_cast<int32_t>(POINT_HORIZON_SEC);
    d.nanosec = static_cast<uint32_t>(
      (POINT_HORIZON_SEC - std::floor(POINT_HORIZON_SEC)) * 1e9);
    pt.time_from_start = d;

    traj.points.clear();
    traj.points.push_back(pt);

    pub_->publish(traj);
  }

private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joints_;
  std::vector<LegDef> legs_;

  std::unordered_map<std::string, double> last_positions_;
  std::deque<std::pair<double,double>> vx_hist_;

  // Logger
  std::ofstream log_file_;

  // IK
  quadbot::LegKinematics ik_;
  double step_length_;
  double swing_height_;

  const double print_period_;
  double last_print_t_;
  bool printed_header_;

  // “Ir a X”
  bool   has_goal_;
  double goal_x_;
  double goal_tolerance_;
  double move_dir_;
  double last_x_;

  // Estado
  bool run_gait_;
  bool have_odom_;

  // Trote (gait libre)
  bool trot_gait_;

  // TCP
  int sockfd_;
  int client_fd_;
  std::unique_ptr<CommandParser> cmd_parser_;

  const double t0_;
};

// -------------- main --------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DutyGaitAuto>());
  rclcpp::shutdown();
  return 0;
}

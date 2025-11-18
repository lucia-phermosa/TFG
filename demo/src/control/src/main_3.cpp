// gait_duty_autoadapt.cpp
// ROS 2 (rclcpp) - Patrón de marcha con “ir a X” y odom por parámetro (default fijo).
// Arranca en postura neutra y SOLO camina al recibir una meta en /goal_x.
// Interruptor opcional: /enable_gait (std_msgs/Bool).

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cmath>
#include <deque>
#include <unordered_map>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <limits>
#include <iomanip>

using std::placeholders::_1;

// ======================= CONFIG FIJA (de la CLI original) =======================
static const char* CONTROLLER_NAME = "joint_trajectory_controller";

static constexpr double FREQUENCY_HZ      = 0.55;
static constexpr double DUTY_STANCE       = 0.60;
static constexpr double PUBLISH_RATE_HZ   = 100.0;
static constexpr double POINT_HORIZON_SEC = 0.14;

static constexpr double DRIVER_AMPL   = 0.32;
static constexpr double AUX_AMPL      = 0.12;
static constexpr double ELBOW_LIFT    = 0.24;
static constexpr double BIAS_SHOULDER = +0.08;
static constexpr double BIAS_HIP_ABD  = 0.06;

static constexpr bool   GEOM_COMP     = true;
static constexpr bool   DO_PRINT      = true;
static constexpr bool   PRINT_ACTUAL  = false;
static constexpr double PRINT_RATE_HZ = 1.0;

// Límites
static constexpr double JOINT_LOWER = -1.57;
static constexpr double JOINT_UPPER = +1.57;
static constexpr double SOFT_MARGIN = 0.12;

// Odometría
static constexpr double VX_WINDOW_SEC    = 2.0;

// “Ir a X”
static constexpr double GOAL_TOLERANCE_DEFAULT = 0.02; // metros

// Direcciones/gain por pata
struct Gains {
  double dir = 1.0;
  double shoulder = 1.0;
  double elbow = 2.5;
};

struct LegDef {
  std::string name;
  int elbow, shoulder, hip; // índices
  double phase;             // fase base
  Gains gains;
};

class DutyGaitAuto : public rclcpp::Node {
public:
  DutyGaitAuto()
  : Node("gait_duty_autoadapt"),
    print_period_(1.0 / std::max(1e-6, PRINT_RATE_HZ)),
    last_print_t_(this->now().seconds()),
    printed_header_(false),
    has_goal_(false),
    goal_x_(0.0),
    goal_tolerance_(GOAL_TOLERANCE_DEFAULT),
    move_dir_(0.0), // 0=indefinido, +1/-1 al fijar meta
    last_x_(std::numeric_limits<double>::quiet_NaN()),
    run_gait_(false),       // arrancar parado
    have_odom_(false)
  {
    // ---------- Parámetro con DEFAULT FIJO del tópico de odometría ----------
    // Cambia el valor por defecto si tu modelo no es "demo".
     std::string odom_topic = this->declare_parameter<std::string>(
      "odom_topic", "/odom");
    // Joints
    joints_.reserve(12);
    for (int i = 1; i <= 12; ++i) joints_.push_back("Revolution_" + std::to_string(i));

    // Patas y fases (trote). RL con gains sobrescritos (como tu CLI).
    legs_ = {
      {"FL", 0, 1, 2, 0.0,  Gains{+1.0, 1.0, 2.5}},
      {"RL", 3, 4, 5, M_PI, Gains{+1.0, 0.40, 7.0}},
      {"RR", 6, 7, 8, 0.0,  Gains{+1.0, 1.0, 2.5}},
      {"FR", 9,10,11, M_PI, Gains{+1.0, 1.0, 2.5}},
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

    // Meta de posición X (m, en marco /odom)
    goal_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/goal_x", 1, std::bind(&DutyGaitAuto::goalCb, this, _1));

    // Interruptor opcional de marcha
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/enable_gait", 1, std::bind(&DutyGaitAuto::enableCb, this, _1));

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1e-6, PUBLISH_RATE_HZ)),
      std::bind(&DutyGaitAuto::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "Gait duty autoadapt: freq=%.3f Hz, duty=%.3f, geom_comp=%s | starts STOPPED",
      FREQUENCY_HZ, DUTY_STANCE, GEOM_COMP ? "true" : "false");
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
    if (x < 0.25) return 4.0 * x;
    else if (x < 0.75) return 2.0 - 4.0 * x;
    else return -4.0 + 4.0 * x;
  }
  static std::tuple<double,bool,double> duty_profile(double phase, double duty) {
    const double twopi = 2.0 * M_PI;
    double ph = std::fmod(phase, twopi);
    if (ph < 0) ph += twopi;
    const double u = ph / twopi;
    if (u < duty) {
      double tau = u / duty;
      double s = -1.0 + tau * 1.0;
      return {s, true, tau};
    } else {
      double tau = (u - duty) / std::max(1e-6, (1.0 - duty));
      double s = 0.0 + tau * 1.0;
      return {s, false, tau};
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
    RCLCPP_INFO(this->get_logger(), "run_gait_ = %s", run_gait_ ? "true" : "false");
    if (!run_gait_) publish_(neutral_posture());
  }

  void goalCb(const std_msgs::msg::Float64::SharedPtr msg) {
    // Ignorar nuevas metas si ya hay una en curso (opcional de seguridad)
    if (has_goal_) {
      RCLCPP_WARN(this->get_logger(), "Ya hay meta activa (%.3f). Ignoro nueva meta: %.3f",
                  goal_x_, msg->data);
      return;
    }
    goal_x_ = msg->data;
    has_goal_ = true;

    if (!have_odom_) {
      run_gait_ = false;  // no andar sin odometría
      move_dir_ = 0.0;
      RCLCPP_WARN(this->get_logger(),
        "Recibí meta X=%.3f pero aún no hay odometría. Quedo parado hasta recibir /odom.", goal_x_);
      return;
    }

    run_gait_ = true;
    const double err = goal_x_ - last_x_;
    move_dir_ = (err >= 0.0) ? +1.0 : -1.0;
    for (auto & L : legs_) L.gains.dir = move_dir_;
    RCLCPP_INFO(this->get_logger(),
      "Nueva meta X=%.3f m | x=%.3f | dir=%+g | tol=%.3f", goal_x_, last_x_, move_dir_, goal_tolerance_);
  }

  double avg_vx() const {
    if (vx_hist_.empty()) return std::numeric_limits<double>::quiet_NaN();
    double s = 0.0;
    for (const auto& p : vx_hist_) s += p.second;
    return s / static_cast<double>(vx_hist_.size());
  }

  // ---------- postura neutra ----------
  std::vector<double> neutral_posture() {
    // hombro = bias, cadera = abducción fija, codo ~ 0
    std::vector<double> q(joints_.size(), 0.0);
    for (const auto& L : legs_) {
      double q_sh = soft_limit(BIAS_SHOULDER * L.gains.shoulder);
      double q_hp = soft_limit(((L.name == "FL" || L.name == "FR") ? +BIAS_HIP_ABD : -BIAS_HIP_ABD));
      double q_el = soft_limit(0.0);
      q[L.shoulder] = q_sh;
      q[L.hip] = q_hp;
      q[L.elbow] = q_el;
    }
    return q;
  }

  // ---------- tick principal ----------
  void tick() {
    const double now_s = this->now().seconds();
    const double t = now_s - t0_;
    const double omega = 2.0 * M_PI * FREQUENCY_HZ;

    // Sin odom: no caminar.
    if (has_goal_ && !have_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
        "Sin odometría (/odom_topic param) → no puedo avanzar hacia la meta. Esperando...");
      publish_(neutral_posture());
      return;
    }

    // Deshabilitado: quieto
    if (!run_gait_) {
      publish_(neutral_posture());
      return;
    }

    // Si hay meta y odom, comprobar llegada
    if (has_goal_ && have_odom_) {
      double err = goal_x_ - last_x_;

      // Fijar/ajustar dirección coherente
      if (move_dir_ == 0.0) {
        move_dir_ = (err >= 0.0) ? +1.0 : -1.0;
        for (auto & L : legs_) L.gains.dir = move_dir_;
      } else {
        for (auto & L : legs_) L.gains.dir = (err >= 0.0) ? +1.0 : -1.0;
      }

      // Parada por llegada
      if (std::fabs(err) <= goal_tolerance_) {
        has_goal_ = false;
        run_gait_ = false;
        RCLCPP_INFO(this->get_logger(), "Objetivo alcanzado: x=%.3f ≈ %.3f (tol=%.3f). Parando.",
                    last_x_, goal_x_, goal_tolerance_);
        publish_(neutral_posture());
        return;
      }
      // Parada si rebasó
      if ((err > 0.0 && move_dir_ < 0.0) || (err < 0.0 && move_dir_ > 0.0)) {
        has_goal_ = false;
        run_gait_ = false;
        RCLCPP_WARN(this->get_logger(),
          "Meta rebasada: x=%.3f, goal=%.3f. Paro para no sobrepasar.", last_x_, goal_x_);
        publish_(neutral_posture());
        return;
      }

      // Log útil (1 Hz)
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
        "goal_x=%.3f  x=%.3f  err=%.3f  tol=%.3f  dir=%+.0f",
        goal_x_, last_x_, err, goal_tolerance_, move_dir_);
    }

    // Generar marcha
    std::vector<double> q(joints_.size(), 0.0);
    const double vx = avg_vx();

    for (const auto& L : legs_) {
      const int elbow = L.elbow, shoulder = L.shoulder, hip = L.hip;
      const double phi = omega * t + L.phase;

      double s; bool in_stance; double tau;
      std::tie(s, in_stance, tau) = duty_profile(phi, DUTY_STANCE);

      // Hombro (driver)
      double q_sh = BIAS_SHOULDER + L.gains.dir * (DRIVER_AMPL * s);
      q_sh *= L.gains.shoulder;

      // Cadera
      double q_hp = (AUX_AMPL * 0.35 * q_sh / std::max(1e-6, DRIVER_AMPL));
      const double abd = (L.name == "FL" || L.name == "FR") ? +BIAS_HIP_ABD : -BIAS_HIP_ABD;

      // Codo
      const double lift = in_stance ? 0.0 : (ELBOW_LIFT * half_cosine(tau));
      const double tri  = triangle((phi / (2.0 * M_PI)) + 0.25);
      double q_el = 0.25 * (q_sh + q_hp) - lift * tri;
      q_el *= L.gains.elbow;

      // Compensación geométrica
      if (GEOM_COMP) {
        const double q_sh_soft = soft_limit(q_sh);
        if (std::fabs(q_sh_soft - q_sh) > 1e-6) {
          const double delta = q_sh - q_sh_soft;
          q_sh = q_sh_soft;
          q_el += 0.6 * delta * L.gains.elbow;
        }
      }

      // clamps
      q_sh = soft_limit(q_sh);
      q_el = soft_limit(q_el);
      q_hp = soft_limit(q_hp + abd);

      q[shoulder] = q_sh;
      q[hip]      = q_hp;
      q[elbow]    = q_el;
    }

    publish_(q);

    // impresión
    if (DO_PRINT) {
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
            const double act = (it == last_positions_.end()) ? std::numeric_limits<double>::quiet_NaN() : it->second;
            const double err = (std::isnan(act) ? std::numeric_limits<double>::quiet_NaN() : (act - cmd));
            std::cout << "t=" << std::fixed << std::setw(6) << std::setprecision(2) << (now_s - t0_)
                      << "  " << std::left << std::setw(14) << name
                      << "  cmd=" << std::showpos << std::fixed << std::setprecision(3) << cmd << std::noshowpos
                      << "   act=" << std::showpos << std::fixed << std::setprecision(3) << act << std::noshowpos
                      << "   err=" << std::showpos << std::fixed << std::setprecision(3) << err << std::noshowpos
                      << std::endl;
          } else {
            std::cout << "t=" << std::fixed << std::setw(6) << std::setprecision(2) << (now_s - t0_)
                      << "  " << std::left << std::setw(14) << name
                      << "  cmd=" << std::showpos << std::fixed << std::setprecision(3) << cmd << std::noshowpos
                      << std::endl;
          }
        }
        if (!std::isnan(vx)) {
          std::cout << "vx̄≈ " << std::showpos << std::fixed << std::setprecision(3) << vx << " m/s" << std::noshowpos
                    << " | goal: " << (has_goal_ ? "activo" : "—")
                    << " | run_gait: " << (run_gait_ ? "ON" : "OFF")
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
    d.nanosec = static_cast<uint32_t>((POINT_HORIZON_SEC - std::floor(POINT_HORIZON_SEC)) * 1e9);
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

  const double t0_ = rclcpp::Clock().now().seconds();

  const double print_period_;
  double last_print_t_;
  bool printed_header_;

  // “Ir a X”
  bool   has_goal_;
  double goal_x_;
  double goal_tolerance_;
  double move_dir_;   // +1 avanzar, -1 retroceder, 0 indefinido
  double last_x_;

  // Estado
  bool run_gait_;
  bool have_odom_;
};

// -------------- main --------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DutyGaitAuto>());
  rclcpp::shutdown();
  return 0;
}

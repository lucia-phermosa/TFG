// gait_duty_autoadapt.cpp
// ROS 2 (rclcpp) - Patrón de marcha con compensación "software" (sin tocar URDF)

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>
#include <deque>
#include <unordered_map>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

static const char* CONTROLLER_NAME = "joint_trajectory_controller";

// ======================= CONFIG FIJA (de tu CLI) =======================
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
static constexpr double PRINT_RATE_HZ = 1.0; // igual que el Python por defecto

// Límites (coinciden con URDF del ejemplo)
static constexpr double JOINT_LOWER = -1.57;
static constexpr double JOINT_UPPER = +1.57;
static constexpr double SOFT_MARGIN = 0.12;

// Auto-inversión por odometría (mantenemos la ventana aunque no invertimos automáticamente)
static constexpr double VX_WINDOW_SEC   = 2.0;
static constexpr double VX_INVERT_THRESH = -0.01;

// Direcciones/gain por pata (por defecto 1.0; overrides para RL como en tu CLI)
struct Gains {
  double dir = 1.0;
  double shoulder = 1.0;
  double elbow = 2.5;
};

struct LegDef {
  std::string name;
  int elbow, shoulder, hip; // índices en el vector de JOINTS
  double phase;             // fase base
  Gains gains;
};

class DutyGaitAuto : public rclcpp::Node {
public:
  DutyGaitAuto()
  : Node("gait_duty_autoadapt"),
    print_period_(1.0 / std::max(1e-6, PRINT_RATE_HZ)),
    last_print_t_(this->now().seconds()),
    printed_header_(false)
  {
    // Joints
    joints_.reserve(12);
    for (int i = 1; i <= 12; ++i) joints_.push_back("Revolution_" + std::to_string(i));

    // Definición de patas y fases (trote: FL/RR en fase, FR/RL a π)
    // Índices [elbow, shoulder, hip] por pata según el Python:
    // FL:[0,1,2], RL:[3,4,5], RR:[6,7,8], FR:[9,10,11]
    legs_ = {
      {"FL", 0, 1, 2, 0.0,   Gains{+1.0, 1.0, 2.5}},
      {"RL", 3, 4, 5, M_PI,  Gains{+1.0, 0.40, 7.0}}, // overrides de tu CLI
      {"RR", 6, 7, 8, 0.0,   Gains{+1.0, 1.0, 2.5}},
      {"FR", 9,10,11, M_PI,  Gains{+1.0, 1.0, 2.5}},
    };

    // Publisher
    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/" + std::string(CONTROLLER_NAME) + "/joint_trajectory", 10);

    // QoS: BEST_EFFORT como en Python
    auto qos_odom = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 20))
                      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    auto qos_js   = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 50))
                      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Subs
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos_odom, std::bind(&DutyGaitAuto::odomCb, this, _1));

    js_sub_   = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", qos_js, std::bind(&DutyGaitAuto::jsCb, this, _1));

    // Timer principal (publish rate)
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1e-6, PUBLISH_RATE_HZ)),
      std::bind(&DutyGaitAuto::tick, this));

    // Log inicial
    RCLCPP_INFO(this->get_logger(),
      "Gait duty autoadapt: freq=%.3f Hz, duty=%.3f, geom_comp=%s | odom=/odom",
      FREQUENCY_HZ, DUTY_STANCE, GEOM_COMP ? "true" : "false");
  }

private:
  // ---------- helpers de forma ----------
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
    // Devuelve s∈[-1,1], in_stance, tau∈[0,1]
    double twopi = 2.0 * M_PI;
    double ph = std::fmod(phase, twopi);
    if (ph < 0) ph += twopi;
    double u = ph / twopi;

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
  }

  void jsCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      last_positions_[msg->name[i]] = msg->position[i];
    }
  }

  double avg_vx() const {
    if (vx_hist_.empty()) return std::numeric_limits<double>::quiet_NaN();
    double s = 0.0;
    for (const auto& p : vx_hist_) s += p.second;
    return s / static_cast<double>(vx_hist_.size());
  }

  // ---------- tick principal ----------
  void tick() {
    const double t = this->now().seconds() - t0_;
    const double omega = 2.0 * M_PI * FREQUENCY_HZ;

    std::vector<double> q(joints_.size(), 0.0);

    const double vx = avg_vx(); // (no invertimos automáticamente nada)

    for (const auto& L : legs_) {
      const int elbow = L.elbow, shoulder = L.shoulder, hip = L.hip;
      const double phi = omega * t + L.phase;

      double s; bool in_stance; double tau;
      std::tie(s, in_stance, tau) = duty_profile(phi, DUTY_STANCE);

      // Hombro (driver) con bias, dirección por pata y ganancia de hombro
      double q_sh = BIAS_SHOULDER + L.gains.dir * (DRIVER_AMPL * s);
      q_sh *= L.gains.shoulder;

      // Cadera acompaña discretamente (abducción fija + pequeño seguidor)
      double q_hp = (AUX_AMPL * 0.35 * q_sh / std::max(1e-6, DRIVER_AMPL));
      const double abd = (L.name == "FL" || L.name == "FR") ? +BIAS_HIP_ABD : -BIAS_HIP_ABD;

      // Codo – más lift en swing, con ganancia grande para “compensar”
      const double lift = in_stance ? 0.0 : (ELBOW_LIFT * half_cosine(tau));
      const double tri  = triangle((phi / (2.0 * M_PI)) + 0.25);
      double q_el = 0.25 * (q_sh + q_hp) - lift * tri;
      q_el *= L.gains.elbow;

      // Compensación geométrica: si hombro se acerca a límite, reducir y pasar parte al codo
      if (GEOM_COMP) {
        const double q_sh_soft = soft_limit(q_sh);
        if (std::fabs(q_sh_soft - q_sh) > 1e-6) {
          const double delta = q_sh - q_sh_soft;
          q_sh = q_sh_soft;
          q_el += 0.6 * delta * L.gains.elbow;
        }
      }

      // clamps finales
      q_sh = soft_limit(q_sh);
      q_el = soft_limit(q_el);
      q_hp = soft_limit(q_hp + abd);

      q[shoulder] = q_sh;
      q[hip]      = q_hp;
      q[elbow]    = q_el;
    }

    publish_(q);

    // impresión periódica
    if (DO_PRINT) {
      const double now_s = this->now().seconds();
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
            std::cout << "t=" << std::fixed << std::setw(6) << std::setprecision(2) << t
                      << "  " << std::left << std::setw(14) << name
                      << "  cmd=" << std::showpos << std::fixed << std::setprecision(3) << cmd << std::noshowpos;
            std::cout << "   act=" << std::showpos << std::fixed << std::setprecision(3) << act << std::noshowpos
                      << "   err=" << std::showpos << std::fixed << std::setprecision(3) << err << std::noshowpos
                      << std::endl;
          } else {
            std::cout << "t=" << std::fixed << std::setw(6) << std::setprecision(2) << t
                      << "  " << std::left << std::setw(14) << name
                      << "  cmd=" << std::showpos << std::fixed << std::setprecision(3) << cmd << std::noshowpos
                      << std::endl;
          }
        }
        if (!std::isnan(vx)) {
          std::cout << "vx̄≈ " << std::showpos << std::fixed << std::setprecision(3) << vx << " m/s" << std::noshowpos
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
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joints_;
  std::vector<LegDef> legs_;

  std::unordered_map<std::string, double> last_positions_;
  std::deque<std::pair<double,double>> vx_hist_;

  const double t0_ = rclcpp::Clock().now().seconds();

  const double print_period_;
  double last_print_t_;
  bool printed_header_;
};

// -------------- main --------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DutyGaitAuto>());
  rclcpp::shutdown();
  return 0;
}

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cstdint>

#include "Body.hpp"
#include "Enums.hpp"

class WalkGait {
public:
  WalkGait();
  ~WalkGait() = default;

  void setBody(Body* b);

  bool hasFinished();

  void pause();
  void resume();

  void setGoalBase(float goal_bx, float goal_by);

  // Firma tal cual la usas en tu JointPubNode
  bool walk(rclcpp::Clock clock_,
            rclcpp::Logger logger,
            bool walk_ON,
            bool &newPose,
            bool haveTarget,
            float targetX,
            float targetY);

public:
  // Estado leído desde fuera (JointPubNode lo usa)
  bool isPaused{true};

private:
  // Dependencias
  Body* body{nullptr};

  // Estado básico
  int  walk_state{0};
  int  pause_state{0};
  int  leg2move{0};

  // Parámetros (los que realmente usa tu Gaits.cpp)
  float stepSize{0.005f};
  float stepX{0.003f};
  float stepY{0.0f};
  float zlift{0.04f};

  int order_idx_{0};

  int hold_ticks{10};
  int hold_ctr{0};

  // Goal (world) usado en setGoalBase()
  bool  goal_active_{false};
  bool  stance_init_{false};
  bool  seq_active_{false};
  float goal_world_x_{0.0f};
  float goal_world_y_{0.0f};

  // Target-following (world) usado en walk()
  bool  target_active_{false};
  bool  target_prev_active_{false};
  float target_x_{0.0f};
  float target_y_{0.0f};
  uint8_t cycle_moved_mask_{0};

  // Self-test / state machine usada en walk()
  SelfTestState self_state_{SelfTestState::IDLE};
  rclcpp::Time  self_next_time_{};

  int self_leg_idx_{0};
  std::array<int, 4> self_order_{{FR, RR, FL, RL}};

  float  self_stepX_{0.03f};
  float  self_stepY_{0.0f};
  float  self_zlift_{0.04f};
  double self_gap_s_{0.0};
};
#pragma once
#include <cmath>
#include <algorithm>
#include "Body.hpp"
#include "Enums.hpp"
#include "rclcpp/rclcpp.hpp"

class WalkGait {
public:
  bool isPaused{true};
  float  stepX{0.0f}, stepY{0.0f};
  int  leg_height{30};

  WalkGait();
  void setBody(Body* b);
  void setSpeed(float velX_percent, float velY_percent);
  void pause();
  void resume();
  bool walk();
  bool walk1();
  bool walk2();
  bool hasFinished();

  template <typename Container>
  static inline Leg* safe_leg(const Container& v, int idx) {
    if (idx < 0) return nullptr;
    using size_type = decltype(v.size());
    if (static_cast<size_type>(idx) >= v.size()) return nullptr;
    auto const& sp = v[static_cast<size_type>(idx)];
    return sp ? sp.get() : nullptr;
  }

  // NUEVOS (opcionales)
  void setLegHeight(int mm);
  void setStepSize(int mm);
  void setHoldTicks(int ticks);

private:
  Body* body{nullptr};

  // Estado
  int  walk_state{0};
  int pause_state{0};
  int  leg2move{0};

  // Parámetros
  float stepSize{50.0f};

  // Secuencia fija
  int  order[4]{FR, RL, FL, RR};
  int  order_idx{0};

  // Pausa de estabilidad
  //int  hold_ticks{10};
  //int  hold_ctr{0};

  int pend_dx = 0, pend_dy = 0, pend_dz = 0, pend_height = 30;
  int hold_ticks = 10, hold_ctr = 0;
};

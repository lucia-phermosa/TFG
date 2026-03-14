#include "Gaits.hpp"

WalkGait::WalkGait()
{
  // Estado
  walk_state = 0;      // 0: listo para lanzar swing | 1: swing en curso | 2: hold/estabilizar
  pause_state = 0;     // (no usado aquí, lo dejamos por compatibilidad)
  isPaused   = true;

  // Parámetros (ajusta a tu gusto o expón setters en el .hpp)
  if (stepSize <= 0) stepSize = 0.005f;  // mm base si no estaba inicializado en el .hpp
  stepX = 0.003f;
  stepY = 0.00f;
  zlift = 0.04f;     // mm de elevación (más natural que 50 para “crawl”)

  order_idx_ = 0;

  // Pausa de estabilidad (nº de ticks de walk() entre pasos)
  hold_ticks = 10;     // ~0.2 s si publicas a 50 Hz
  hold_ctr   = 0;

}

void WalkGait::setBody(Body *b) { body = b; }

bool WalkGait::hasFinished() { return (walk_state == 0) && isPaused; }

void WalkGait::pause()  { isPaused = true;  }
void WalkGait::resume() { isPaused = false; }

static inline float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}
static inline float wrapPi(float a) {
  while (a >  static_cast<float>(M_PI)) a -= 2.f * static_cast<float>(M_PI);
  while (a < -static_cast<float>(M_PI)) a += 2.f * static_cast<float>(M_PI);
  return a;
}
static inline float hypot2f(float x, float y) {
  return std::sqrt(x*x + y*y);
}

void WalkGait::setGoalBase(float goal_bx, float goal_by)
{
  // convertir base_link -> world usando yaw actual
  const float yaw = body->yaw;

  const float c = std::cos(yaw);
  const float s = std::sin(yaw);

  const float gx_w = goal_bx * c - goal_by * s;
  const float gy_w = goal_bx * s + goal_by * c;

  goal_world_x_ = body->worldX + gx_w;
  goal_world_y_ = body->worldY + gy_w;

  goal_active_ = true;
  stance_init_ = false;
  seq_active_  = false;
  order_idx_   = 0;
  walk_state   = 0;
}

bool WalkGait::walk(rclcpp::Clock clock_,
                    rclcpp::Logger logger,
                    bool walk_ON,
                    bool &newPose,
                    bool haveTarget,   // NUEVO
                    float targetX,     // NUEVO (WORLD)
                    float targetY)     // NUEVO (WORLD)
{
  newPose = false;

  if (!walk_ON) {
    isPaused = true;
    return false;
  }
  isPaused = false;

  const auto now = clock_.now();

  // -------- Target on/off --------
  if (haveTarget) {
    target_active_ = true;
    target_x_ = targetX;
    target_y_ = targetY;
  } else {
    target_active_ = false;
  }

  // Si activas target mode justo ahora, reinicia ciclo y estados
  if (target_active_ && !target_prev_active_) {
    cycle_moved_mask_ = 0;         // 4 bits: patas movidas en este ciclo
    self_state_ = SelfTestState::IDLE;
  }
  target_prev_active_ = target_active_;

  // -------- Comandos stepX/stepY --------
  float stepX_cmd = self_stepX_;
  float stepY_cmd = self_stepY_;

  // Pose del body en WORLD (ajusta nombres si difieren)
  const float bx  = body->worldX;
  const float by  = body->worldY;
  const float yaw = body->yaw;     // IMPORTANT: yaw actual del body en WORLD

  // Si hay target: decide si girar o ir recto
  if (target_active_) {
    const float dx = target_x_ - bx;
    const float dy = target_y_ - by;
    const float dist = hypot2f(dx, dy);

    // Llegada al target
    const float pos_tol = 0.02f; // 2 cm (ajusta)
    if (dist < pos_tol) {
      self_state_ = SelfTestState::IDLE;
      cycle_moved_mask_ = 0;
      return false;
    }

    // Rumbo deseado y error de yaw
    const float desired_heading = std::atan2(dy, dx);
    const float yaw_err = wrapPi(desired_heading - yaw);

    // Parámetros giro
    const float yaw_tol = 2.0f * static_cast<float>(M_PI) / 180.0f; // 2 deg
    const float turn_stepY = 0.01f;                    // pequeño (ajusta)
    const float forward_when_turning = 0.0f;           // o 0.005f si quieres avanzar girando

    // StepX “hacia target” (capado)
    const float stepX_max = self_stepX_;
    stepX_cmd = clampf(dist, 0.0f, stepX_max);

    if (std::fabs(yaw_err) > yaw_tol) {
      // No orientado aún: gira con stepY pequeño
      stepY_cmd = (yaw_err > 0.f) ? +turn_stepY : -turn_stepY;
      stepX_cmd = forward_when_turning;
    } else {
      // Orientado: ir recto
      stepY_cmd = 0.f;
    }
  }

  // -------- Selector de pata --------
  auto choose_leg = [&]() -> int {
    // Caso 1: sin target -> orden fijo
    if (!target_active_) {
      return self_order_[self_leg_idx_];
    }

    // Caso 2: con target -> más alejada del target, sin repetir en el ciclo
    if (cycle_moved_mask_ == 0x0F) {
      cycle_moved_mask_ = 0; // nuevo ciclo de 4
    }

    int bestLeg = -1;
    float bestD2 = -1.f;

    const float c = std::cos(yaw);
    const float s = std::sin(yaw);

    for (int i = 0; i < 4; ++i) {
      if (cycle_moved_mask_ & (1u << i)) continue; // ya movida en este ciclo

      // Pie medido en base_link (tu caso)
      const float fx_b = body->legs_[i]->foot_meas_x;
      const float fy_b = body->legs_[i]->foot_meas_y;

      // Transform a WORLD: p_world = body_world + Rz(yaw) * p_base
      const float fx_w = bx + (c * fx_b - s * fy_b);
      const float fy_w = by + (s * fx_b + c * fy_b);

      const float dx = target_x_ - fx_w;
      const float dy = target_y_ - fy_w;
      const float d2 = dx*dx + dy*dy;

      if (d2 > bestD2) {
        bestD2 = d2;
        bestLeg = i;
      }
    }

    // Fallback seguro
    if (bestLeg < 0) bestLeg = self_order_[0];
    return bestLeg;
  };

  // -------- Máquina de estados --------
  switch (self_state_)
  {
    case SelfTestState::IDLE:
    {
      self_leg_idx_ = 0;
      self_state_ = SelfTestState::LEG_START;
      return false;
    }

    case SelfTestState::LEG_START:
    {
      // Sin target: ciclo fijo de 4 patas
      if (!target_active_) {
        if (self_leg_idx_ >= 4) {
          self_leg_idx_ = 0;
          self_state_ = SelfTestState::LEG_START;
          return false;
        }
      }
      // Con target: el ciclo lo controla cycle_moved_mask_ (no hace falta idx)

      self_state_ = SelfTestState::LEG_RUN;
      return false;
    }

    case SelfTestState::LEG_RUN:
    {
      const int leg = choose_leg();

      const bool done = body->moveFoot(
        leg,
        stepX_cmd, stepY_cmd, 0.005f,
        self_zlift_,
        incremental
      );

      newPose = true;

      if (done) {
        if (!target_active_) {
          self_leg_idx_++;
        } else {
          cycle_moved_mask_ |= (1u << leg); // marca pata movida en este ciclo
        }

        self_next_time_ = now + rclcpp::Duration::from_seconds(self_gap_s_);
        self_state_ = SelfTestState::GAP;
      }
      return false;
    }

    case SelfTestState::GAP:
    {
      newPose = true;
      if (now < self_next_time_) return false;

      self_state_ = SelfTestState::LEG_START;
      return false;
    }

    default:
      return false;
  }
}

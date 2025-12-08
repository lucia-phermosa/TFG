#include "Gaits.hpp"

// ------------------------------------------------------------
// Implementación crawl clásico (una pata en swing, 3 en apoyo)
// Secuencia fija: FR → RL → FL → RR
// ------------------------------------------------------------

WalkGait::WalkGait()
{
  // Estado
  walk_state = 0;      // 0: listo para lanzar swing | 1: swing en curso | 2: hold/estabilizar
  pause_state = 0;     // (no usado aquí, lo dejamos por compatibilidad)
  isPaused   = true;

  // Parámetros (ajusta a tu gusto o expón setters en el .hpp)
  if (stepSize <= 0) stepSize = 50;  // mm base si no estaba inicializado en el .hpp
  stepX = 0;
  stepY = 0;
  leg_height = 30;     // mm de elevación (más natural que 50 para “crawl”)

  // Secuencia fija
  order[0] = FR;
  order[1] = RL;
  order[2] = FL;
  order[3] = RR;
  order_idx = 0;

  // Pausa de estabilidad (nº de ticks de walk() entre pasos)
  hold_ticks = 10;     // ~0.2 s si publicas a 50 Hz
  hold_ctr   = 0;

  // Objetivo de swing pendiente (se reaplica cada tick en state==1)
  pend_dx = pend_dy = pend_dz = 0;
  pend_height = leg_height;

  leg2move = order[order_idx];
}

void WalkGait::setBody(Body *b) { body = b; }

// velX_percent y velY_percent en % de stepSize
void WalkGait::setSpeed(float velX_percent, float velY_percent) {
  // stepSize lo declaras en tu Gaits.hpp; aquí solo lo usamos
  stepX = static_cast<float>(std::round(stepSize * velX_percent / 100.0f));
  stepY = static_cast<float>(std::round(stepSize * velY_percent / 100.0f));
}

bool WalkGait::hasFinished() { return (walk_state == 0) && isPaused; }

void WalkGait::pause()  { isPaused = true;  }
void WalkGait::resume() { isPaused = false; }

bool WalkGait::walk() {
  if (walk_state == 0) {
    if (!isPaused) walk_state = 1;
  }
  else if (walk_state == 1) {
    float mx=0.f, my=0.f, mz=0.f;
    body->getMeanFootPos(&mx, &my, &mz);

    float max_dist = -1.0f;
    int   best_leg = -1;

    const auto& legs = body->legs();  // std::array<shared_ptr<Leg>, 4>

    for (int i = 0; i < static_cast<int>(legs.size()) && i < 4; ++i) {
      Leg* leg = safe_leg(legs, i);   // <-- ahora acepta array o vector
      if (!leg) continue;

      float dX=0.f, dY=0.f;
      leg->getFoot2ReffVect(mx, my, &dX, &dY);
      const float dxs  = dX + stepX;
      const float dys  = dY + stepY;
      const float dist = std::sqrt(dxs*dxs + dys*dys);

      if (dist > max_dist) { max_dist = dist; best_leg = i; }
    }

    if (best_leg < 0) return false;  // no hay patas válidas aún

    leg2move   = best_leg;
    walk_state = 2;
  }
  else if (walk_state == 2) {
    float mx=0.f, my=0.f, mz=0.f;
    body->getMeanFootPos(&mx, &my, &mz);

    const auto& legs = body->legs();
    Leg* leg = safe_leg(legs, leg2move);
    if (!leg) { walk_state = 0; return false; }

    float dX=0.f, dY=0.f;
    leg->getFoot2ReffVect(mx, my, &dX, &dY);

    // printf("[GAIT] leg2move: %d, dX+stepX: %f, dY+stepY: %f, leg_height: %d\n",
    //         leg2move, dX+stepX, dY+stepY, leg_height);

    if (body->moveFoot(leg2move, dX + stepX, dY + stepY, 0, leg_height, incremental, false)) {
      walk_state = 0;
      return true;
    }
  }
  return false;
}

bool WalkGait::walk1() {
  if (!body) return false;

  // --- OSCILADOR DE PRUEBA: fuerza cambios en las juntas ---
  {
    static double t = 0.0;
    t += 0.02; // ~50 Hz si el timer es 50 Hz
    for (auto& sp : body->legs()) {
      if (!sp) continue;
      auto q = sp->joint_positions_rad();   // [H, B, A]
      q[0] = 0.20 * std::sin(t + 0.0);      // hombro
      q[1] = 0.35 * std::sin(t + 1.0);      // brazo
      q[2] = 0.40 * std::sin(t + 2.0);      // antebrazo
      sp->set_joint_positions_rad(q);       // <-- ¡esto cambia /joint_states!
    }
  }
  // ---------------------------------------------------------

  // Lógica original del estado (no hace daño dejarla, pero ahora ya hay movimiento garantizado)
  if (walk_state == 0) {
    if (!isPaused) walk_state = 1;
  } else if (walk_state == 1) {
    float dist[4];
    for (int i = 0; i < 4; i++) {
      float x, y, z; 
      body->getMeanFootPos(&x,&y,&z);
      float dX=0.0f, dY=0.0f; 
      if (auto* leg = body->leg_at(i)) {
        leg->getFoot2ReffVect(x, y, &dX, &dY);
      } else {
        dist[i] = 0;
        continue;
      }
      dist[i] = std::abs(dX + stepX) + std::abs(dY + stepY);
    }
    int legID = 0;
    float best = dist[0];
    for (int i = 1; i < 4; ++i) if (dist[i] > best){ best = dist[i]; legID = i; }
    leg2move = legID;
    walk_state = 2;
  } else if (walk_state == 2) {
    float x, y, z; 
    body->getMeanFootPos(&x,&y,&z);
    float dX=0.0f, dY=0.0f; 
    if (auto* leg = body->leg_at(leg2move)) {
      leg->getFoot2ReffVect(x, y, &dX, &dY);
    }
    if (body->moveFoot(leg2move, dX + stepX, dY + stepY, 0, leg_height, incremental, false)) {
      walk_state = 0;
      return true;
    }
  }
  return false;
}

// Lanza y sigue un swing hasta terminar. Devuelve true justo cuando termina un paso.
// bool WalkGait::walk2() {
//   if (!body || isPaused) return false;

//   // ---------- Estado 0: elegir pata y fijar objetivo ----------
//   if (walk_state == 0) {
//     leg2move = order[order_idx];

//     // Centro (media) de pies actuales
//     float cx=0.0f, cy=0.0f, cz=0.0f;
//     body->getMeanFootPos(&cx, &cy, &cz);

//     // Vector del pie a la referencia (relativo)
//     float dX = 0.0f, dY = 0.0f;
//     if (auto* leg = body->leg_at(static_cast<size_t>(leg2move))) {
//       leg->getFoot2ReffVect(cx, cy, &dX, &dY);
//     }

//     // Guardamos objetivo incremental (avance +X/Y) y altura
//     pend_dx    = dX + stepX;
//     pend_dy    = dY + stepY;
//     pend_dz    = 0;
//     pend_height= leg_height;

//     // Primer disparo del swing
//     (void)body->moveFoot(leg2move, pend_dx, pend_dy, pend_dz, pend_height, incremental, false);

//     walk_state = 1; // swing en curso
//     return false;
//   }

//   // ---------- Estado 1: swing en curso (reaplicar el MISMO objetivo cada tick) ----------
//   if (walk_state == 1) {
//     const bool finished = body->moveFoot(leg2move, pend_dx, pend_dy, pend_dz, pend_height, incremental, false);

//     // Parche visual: mientras dura el swing, escribe ángulos en esa pata para que RViz muestre movimiento.
//     if (auto* leg = body->leg_at(static_cast<size_t>(leg2move))) {
//       auto q = leg->joint_positions_rad(); // [H, B, A]
//       static double t = 0.0; t += 0.02;    // ~50 Hz
//       q[0] = 0.15 * std::sin(t + 0.0);     // hombro
//       q[1] = 0.25 * std::sin(t + 0.7);     // brazo
//       q[2] = 0.30 * std::sin(t + 1.4);     // antebrazo
//       leg->set_joint_positions_rad(q);
//     }

//     if (finished) {
//       walk_state = 2;  // pasar a hold/estabilizar
//       hold_ctr   = 0;
//       return true;     // reporta “paso completado”
//     }
//     return false;
//   }

//   // ---------- Estado 2: pequeña pausa de estabilidad antes del siguiente swing ----------
//   if (walk_state == 2) {
//     if (++hold_ctr >= hold_ticks) {
//       order_idx = (order_idx + 1) % 4;   // siguiente pata de la secuencia
//       walk_state = 0;                    // listo para lanzar otro swing
//     }
//     return false;
//   }

//   return false;
// }
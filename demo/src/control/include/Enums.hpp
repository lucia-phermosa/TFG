#pragma once

enum MovementType { absolute = 0, incremental = 1 };
enum LegID {FR = 0, RR = 1, RL = 2, FL = 3};
enum Side { left = -1, right = 1 };
enum End  { front = -1, rear  = 1 };
enum Joint { elbow = 0, sh = 1, hip = 2 };
enum class SelfTestState {
  IDLE,
  LEG_START,
  LEG_RUN,
  GAP,
  DONE
};

struct Vec3 {
  float x{0.f}, y{0.f}, z{0.f};
};

struct V2 { float x, y; };

struct LegOffsets {
  Vec3 t_sh;  // Hombro -> Brazo
  Vec3 t_el;  // Brazo  -> Antebrazo
  Vec3 t_ft;  // Antebrazo -> Foot
  bool valid{false};
};
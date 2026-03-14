#include "Planner.hpp"

TrayectoryPlanner::TrayectoryPlanner(rclcpp::Clock::SharedPtr clock)
: clock_(std::move(clock)), start_time_(clock_->now())
{}

void TrayectoryPlanner::setPlanner(const rclcpp::Logger& logger_, float movementSpeed,
                                   float startPx, float startPy, float startPz,
                                   float finalX, float finalY, float finalZ)
{
  speed_ = std::max(1e-6f, movementSpeed);

  startX  = startPx;  startY  = startPy;  startZ  = startPz;
  targetX = finalX;   targetY = finalY;   targetZ = finalZ;

  // RCLCPP_INFO(logger_, "Start (%.3f, %.3f, %.3f) Target (%.3f, %.3f, %.3f)", startX, startY, startZ, targetX, targetY, targetZ);

  double dist =
    std::abs(targetX - startX) +
    std::abs(targetY - startY) +
    std::abs(targetZ - startZ);

  // tiempo en segundos
  const double min_time_s = 0.15;   // <- ajusta: 0.15–0.30 suele ir bien
  const double max_time_s = 2.0;    // por seguridad

  targetTime_s = dist / speed_;
  if (std::isnan(targetTime_s) || std::isinf(targetTime_s)) targetTime_s = min_time_s;
  targetTime_s = std::clamp(targetTime_s, min_time_s, max_time_s);

  // velocidades en "unidades por segundo"
  velX = (targetX - startX) / static_cast<float>(targetTime_s);
  velY = (targetY - startY) / static_cast<float>(targetTime_s);
  velZ = (targetZ - startZ) / static_cast<float>(targetTime_s);

  restartTimer();
  active_ = true;
}

void TrayectoryPlanner::restartTimer()
{
  start_time_ = clock_->now();
}

bool TrayectoryPlanner::computeNewPos()
{
  if (!active_) {
    newX = targetX; newY = targetY; newZ = targetZ;
    return true;
  }

  rclcpp::Duration elapsed = clock_->now() - start_time_;
  double t = std::min(elapsed.seconds(), targetTime_s);

  newX = startX + velX * static_cast<float>(t);
  newY = startY + velY * static_cast<float>(t);
  newZ = startZ + velZ * static_cast<float>(t);

  if (elapsed.seconds() >= targetTime_s) {
    newX = targetX; newY = targetY; newZ = targetZ;
    active_ = false;
    return true;
  }
  return false;
}

FootPlanner::FootPlanner()
: planner(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
{
  state = 0;
}

FootPlanner::FootPlanner(rclcpp::Clock::SharedPtr clock)
: planner(std::move(clock))
{
  state = 0;
}

void FootPlanner::setPlanner(const rclcpp::Logger& logger_, float movementSpeed, float px_, float py_, float pz_, float tx_, float ty_, float tz_, float dz_){
  speed = std::max(1e-3f, movementSpeed);

  px = px_; 
  py = py_; 
  pz = pz_;
  tx = tx_;
  ty = ty_; 
  tz = tz_;

  newX = px; newY = py; newZ = pz;

  // Tramo 1: subir (Z hacia arriba) manteniendo XY
  float lift_target = pz + dz;
  planner.setPlanner(logger_, speed, px, py, pz, tx, ty, lift_target);
  // RCLCPP_INFO(logger_, "[FP set] px=%.4f py=%.4f pz=%.4f tx=%.4f ty=%.4f tz=%.4f", px, py, pz, tx, ty, lift_target);
  state = 1;
}


bool FootPlanner::updatePos(const rclcpp::Logger& logger_)
{
  if (state == 0) {
    newX = tx; newY = ty; newZ = tz;
    return true;
  }

  // --- 3 tramos ---
  const bool done = planner.computeNewPos();
  newX = planner.newX;
  newY = planner.newY;
  newZ = planner.newZ;

  if (!done) return false;

  // Tramo 1 terminado -> Tramo 2: avanzar manteniendo Z alta (tz + dz)
  if (state == 1) {
    float lift_target = tz + dz;
    planner.setPlanner(logger_, speed, newX, newY, newZ, tx, ty, lift_target);
    // RCLCPP_INFO(logger_, "[State1] newX=%.4f newY=%.4f newZ=%.4f tx=%.4f ty=%.4f tz=%.4f", newX, newY, newZ, tx, ty, lift_target);
    state = 2;
    return false;
  }

  // Tramo 2 terminado -> Tramo 3: bajar a tz (touchdown)
  if (state == 2) {
    planner.setPlanner(logger_, speed, newX, newY, newZ, tx, ty, tz);
    // RCLCPP_INFO(logger_, "[State2] newX=%.4f newY=%.4f newZ=%.4f tx=%.4f ty=%.4f tz=%.4f", newX, newY, newZ, tx, ty, tz);
    state = 3;
    return false;
  }

  if (state == 3)
  {
    newX = tx; newY = ty; newZ = tz;
    state = 0;
    return true;
  }

  // Tramo 3 terminado -> listo
  state = 0;
  newX = tx; newY = ty; newZ = tz;
  return true;
  
}


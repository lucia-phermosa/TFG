#include "Planner.hpp"

TrayectoryPlanner::TrayectoryPlanner(rclcpp::Clock::SharedPtr clock)
: clock_(std::move(clock)), start_time_(clock_->now())
{}

void TrayectoryPlanner::setPlanner(float movementSpeed, float startPx, float startPy, float startPz,
                                   float finalX, float finalY, float finalZ){
  speed_ = std::max(1e-3f, movementSpeed);

  startX = startPx; 
  startY = startPy; 
  startZ = startPz;
  targetX = finalX; 
  targetY = finalY; 
  targetZ = finalZ;

  // Simple time estimate: distance L1 divided by speed
  float dist = std::abs(targetX - startX) + std::abs(targetY - startY) + std::abs(targetZ - startZ);
  targetTime_ms = std::max(1, int(1000.0f * (dist / speed_)));
  
  velX = (float)(finalX - startX) / (float)targetTime_ms;
  velY = (float)(finalY - startY) / (float)targetTime_ms;
  velZ = (float)(finalZ - startZ) / (float)targetTime_ms;

  restartTimer();
}

void TrayectoryPlanner::restartTimer(){
  start_time_ = clock_->now();
}

bool TrayectoryPlanner::computeNewPos(){
  rclcpp::Duration elapsed = clock_->now() - start_time_;

  // milisegundos en double (evita división entera y pérdida de precisión)
  const double ms = static_cast<double>(elapsed.nanoseconds()) / 1'000'000.0;

  newX = startX + (float) (velX * ms);
  newY = startY + (float) (velY * ms);
  newZ = startZ + (float) (velZ * ms);

  if (ms >= targetTime_ms) {
    newX = targetX;
    newY = targetY;
    newZ = targetZ;
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

void FootPlanner::setPlanner(float movementSpeed, float px_, float py_, float pz_, float tx_, float ty_, float tz_, int dz){
    speed = movementSpeed;
    px = px_;
    py = py_;
    pz = pz_;
    tx = tx_;
    ty = ty_;
    tz = tz_;
    dZ = static_cast<float>(dz);
    state = 1;
    planner.setPlanner(speed, px, py, pz, px, py, pz+dZ);
}

bool FootPlanner::updatePos(){
    if (state == 1){
        if(planner.computeNewPos()){
            state = 2;
            planner.setPlanner(speed, px, py, pz+dZ, tx, ty, tz+dZ);
        }
    }
    else if (state == 2){
        if(planner.computeNewPos()){
            state = 3;
            planner.setPlanner(speed, tx, ty, tz+dZ, tx, ty, tz);
        }
    }
    else if (state == 3){
        if(planner.computeNewPos()){
            state = 0;
        }
    }
    newX = planner.newX;
    newY = planner.newY;
    newZ = planner.newZ;

    if (state == 0){
        return true;
    }
    return false;
}

#pragma once
#include <cstdint>
#include <chrono>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/clock.hpp>

class TrayectoryPlanner{
public:
  TrayectoryPlanner(rclcpp::Clock::SharedPtr clock);

  void setPlanner(float movementSpeed, float startPx, float startPy, float startPz,
                  float finalX, float finalY, float finalZ);
  void restartTimer();
  bool computeNewPos();

  bool targetTime_ms = 1000.0;
  float startX = 0.0f, startY = 0.0f, startZ = 0.0f;
  float targetX = 0.0f, targetY = 0.0f, targetZ = 0.0f;
  float newX = 0.0f, newY = 0.0f, newZ = 0.0f;
  float velX = 0.0, velY = 0.0, velZ = 0.0;

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time start_time_;
  float speed_ = 1.0f; // units per second (arbitrary)
};

class FootPlanner{
    
    float speed; //in mm/ms
    int targetTime;
    int state;    
public:
    TrayectoryPlanner planner;
    float px, py, pz;
    float tx, ty, tz;
    float dZ;
    float newX, newY, newZ;

    FootPlanner();
    FootPlanner(rclcpp::Clock::SharedPtr clock);
    void setPlanner(float movementSpeed, float px_, float py_, float pz_, float tx_, float ty_, float tz_, int dz);
    bool updatePos();
};

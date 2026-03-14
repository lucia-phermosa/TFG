#pragma once

#include <algorithm>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

class TrayectoryPlanner {
public:
  explicit TrayectoryPlanner(rclcpp::Clock::SharedPtr clock);

  void setPlanner(const rclcpp::Logger& logger_,
                  float movementSpeed,
                  float startPx, float startPy, float startPz,
                  float finalX, float finalY, float finalZ);

  void restartTimer();
  bool computeNewPos();
  bool isActive() const { return active_; }

public:
  bool active_{false};

  double targetTime_s{0.0};

  float startX{0.0f};
  float startY{0.0f};
  float startZ{0.0f};

  float targetX{0.0f};
  float targetY{0.0f};
  float targetZ{0.0f};

  float newX{0.0f};
  float newY{0.0f};
  float newZ{0.0f};

  float velX{0.0f};
  float velY{0.0f};
  float velZ{0.0f};

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time start_time_{};

  float speed_{1.0f};
};

class FootPlanner {
public:
  FootPlanner();
  explicit FootPlanner(rclcpp::Clock::SharedPtr clock);

  void setPlanner(const rclcpp::Logger& logger_,
                  float movementSpeed,
                  float px_, float py_, float pz_,
                  float tx_, float ty_, float tz_,
                  float dz_);

  bool updatePos(const rclcpp::Logger& logger_);

public:
  TrayectoryPlanner planner;

  int state{0};

  float px{0.0f};
  float py{0.0f};
  float pz{0.0f};

  float tx{0.0f};
  float ty{0.0f};
  float tz{0.0f};

  float dz{0.0f};

  float newX{0.0f};
  float newY{0.0f};
  float newZ{0.0f};

private:
  float speed{0.0f};     // mm/ms
  int targetTime{0};
};
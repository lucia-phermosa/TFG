#pragma once

#include <algorithm>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "MamdaniFuzzy.hpp"

struct PIDRateState {
  float kp{2.0f};
  float ki{0.1f};
  float kd{0.1f};

  float i{0.0f};
  float prev_e{0.0f};
  float d_filt{0.0f};
};

class Horizontal_controller {
public:
  // Estado/modos (leídos desde fuera)
  bool hold_ON{false};
  bool cg_control{false};

  // Estimación de “suelo” (rad en tu .cpp)
  float floor_roll{0.0f};
  float floor_pitch{0.0f};

  // Centro de gravedad (salidas)
  float cg_x{0.0f};
  float cg_y{0.0f};

  // API (la que usas en el resto del proyecto)
  void init();
  void resetControllers();

  // model_* y imu_* en rad (según tu .cpp actual)
  void newRobotOrient(float model_roll,
                      float model_pitch,
                      float height,
                      float imu_roll,
                      float imu_pitch);

  // refs/reales en rad; salida roll/pitch en rad (saturado)
  void stabilize(float roll_ref, float pitch_ref,
                 float roll_real, float pitch_real,
                 float* roll, float* pitch);

  void resetPID();

  void stabilizePID(const rclcpp::Logger& logger_,
                    float roll_ref, float pitch_ref,
                    float roll_real, float pitch_real,
                    float* roll_out, float* pitch_out);

private:
  // Controladores difusos
  MamdaniFuzzy roll_control{};
  MamdaniFuzzy pitch_control{};

  // Buffers circulares (15) 
  std::array<float, 15> pitch_ringBuffer{}; 
  unsigned int pitch_bufferPos = 0; 
  std::array<float, 15> roll_ringBuffer{}; 
  unsigned int roll_bufferPos = 0;

  // PID de rate + acumulación a ángulo
  PIDRateState roll_pid_rate_{};
  PIDRateState pitch_pid_rate_{};

  // Referencias (grados) 
  float roll_reff = 0.0f; 
  float pitch_reff = 0.0f;

  float roll_int{0.0f};
  float pitch_int{0.0f};

  float roll_sign_{-1.0f};
  float pitch_sign_{-1.0f};

  std::chrono::steady_clock::time_point last_time{std::chrono::steady_clock::now()};
  double sampleTime_s{0.0};
};
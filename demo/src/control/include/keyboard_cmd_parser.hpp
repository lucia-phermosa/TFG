#pragma once

#include <string>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

class CommandParser {
public:
  explicit CommandParser(
      int socket_fd,
      const rclcpp::Logger& logger =
          rclcpp::get_logger("CommandParser"));

  // Lee del socket hasta ensamblar un comando
  bool readNext();

  // Último resultado parseado
  int last_code() const { return last_code_; }
  const std::string& last_label() const { return last_label_; }
  const std::string& last_msg() const { return msg_; }

  // Reprocesar manualmente el último label
  int dispatchLast() { return parseinputdata(last_label_.c_str()); }

  // -------- Estado expuesto (equivalente al "port" Arduino) --------
  float pX{0.0f};
  float pY{0.0f};
  float pZ{0.0f};

  int walkMode{0};

  int joint{0};
  int joint_offset{0};
  int joint_angle{0};
  int leg_id{0};

  int pitch{0};
  int roll{0};
  int yaw{0};

  int telem{0};
  int hold_orient{0};
  int cg_control{0};

  float camber{0.0f};

  int velX{0};
  int velY{0};

  int trot_gait{0};

  float osc_amp{0.0f};
  float osc_w{0.0f};
  float osc_phi{0.0f};

  float oscX_amp{0.0f};
  float oscY_amp{0.0f};
  float oscX_phi{0.0f};
  float oscY_phi{0.0f};

private:
  // Procesa label usando msg_ ya cargado
  int parseinputdata(const char* label);

  // Socket
  int sockfd_{-1};

  // ROS logger
  rclcpp::Logger logger_;

  // RX state
  bool rxStarted_{false};
  std::string buffer_;
  std::string last_label_;
  std::string msg_;

  int last_code_{0};

  // Procesamiento byte a byte
  bool feedByte(char c);
};
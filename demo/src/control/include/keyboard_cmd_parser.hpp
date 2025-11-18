// CommandParser.hpp
#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <functional>


// ROS 2
#include <rclcpp/rclcpp.hpp>

class CommandParser {
public:
  explicit CommandParser(int socket_fd,
                         const rclcpp::Logger& logger = rclcpp::get_logger("CommandParser"));

  // Lee del socket hasta ensamblar un comando; devuelve true si obtuvo uno.
  bool readNext();

  // Último código devuelto por parseinputdata (válido tras readNext()==true)
  int last_code() const { return last_code_; }
  const std::string& last_label() const { return last_label_; }
  const std::string& last_msg() const { return msg_; }

  // Dispatcher accesible si quieres forzar reprocesado (normalmente no hace falta)
  int dispatchLast() { return parseinputdata(last_label_.c_str()); }

  // --------- Estado público (equivalente al "port" de Arduino) ---------
  float pX=0.0f, pY=0.0f, pZ=0.0f;
  int servoMode=0;
  int walkMode=0;
  int joint=0, joint_offset=0, joint_angle=0;
  int leg_id=0;
  int lowerLimit=0, upperLimit=0;
  int pitch=0, roll=0, yaw=0;
  int telem=0;
  int hold_orient=0;
  int cg_control=0;
  float camber=0.0f;
  int velX=0, velY=0;
  int trot_gait=0;
  float osc_amp=0.0f, osc_w=0.0f, osc_phi=0.0f;
  float oscX_amp=0.0f, oscY_amp=0.0f;
  float oscX_phi=0.0f, oscY_phi=0.0f;

private:
  // Procesa label usando el payload ya almacenado en msg_; retorna código 1..21/-1/0
  int parseinputdata(const char* label);

  // Internos
  int sockfd_;
  rclcpp::Logger logger_;
  bool rxStarted_ = false;
  std::string buffer_;     // entre '#' y ';' (sin delimitadores)
  std::string last_label_; // etiqueta extraída antes del primer ':'
  std::string msg_;        // texto crudo tras el primer ':' (sin '#', sin ';')
  int last_code_ = 0;

  bool feedByte(char c); // true si cierra un comando
};
// CommandParser.cpp
#include "keyboard_cmd_parser.hpp"
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <cstdlib>

CommandParser::CommandParser(int socket_fd, const rclcpp::Logger& logger)
: sockfd_(socket_fd), logger_(logger) {
  buffer_.reserve(128);
}

bool CommandParser::feedByte(char c) {
  if (!rxStarted_) {
    if (c == '#') {
      rxStarted_ = true;
      buffer_.clear();
    }
    return false;
  } else {
    if (c == ';') {
      // Tenemos un frame completo en buffer_: "LABEL[:payload]"
      // Log del comando crudo:
      RCLCPP_INFO(logger_, "Comando recibido: #%s;", buffer_.c_str());

      // Separar label y msg (tras el primer ':', si existe)
      last_label_.clear();
      msg_.clear();

      size_t sep = buffer_.find(':');
      if (sep == std::string::npos) {
        last_label_ = buffer_;
      } else {
        last_label_ = buffer_.substr(0, sep);
        if (sep + 1 < buffer_.size())
          msg_ = buffer_.substr(sep + 1);
      }

      // Quitar posibles espacios/CR/LF en extremos
      auto trim = [](std::string &s){
        while (!s.empty() && (s.back()=='\r' || s.back()=='\n' || s.back()==' ' || s.back()=='\t')) s.pop_back();
        size_t i=0; while (i<s.size() && (s[i]==' '||s[i]=='\t')) ++i; if (i) s.erase(0,i);
      };
      trim(last_label_);
      trim(msg_);

      // Llamar a la función estilo Arduino
      last_code_ = parseinputdata(last_label_.c_str());

      // Preparar para el siguiente frame
      rxStarted_ = false;
      buffer_.clear();
      return true;
    } else if (c == '#') {
      // Reinicio de frame
      buffer_.clear();
      rxStarted_ = true;
    } else {
      buffer_.push_back(c);
    }
    return false;
  }
}

bool CommandParser::readNext() {
  char tmp[256];
  for (;;) {
    ssize_t n = ::recv(sockfd_, tmp, sizeof(tmp), 0);
    if (n > 0) {
      for (ssize_t i = 0; i < n; ++i) {
        if (feedByte(tmp[i])) {
          // se cerró un comando y se llamó a parseinputdata()
          return true;
        }
      }
      continue; // seguir leyendo más trozos si vienen en el mismo recv
    } else if (n == 0) {
      // peer cerró
      return false;
    } else {
      if (errno == EINTR) continue;
      if (errno == EWOULDBLOCK || errno == EAGAIN) {
        // no hay datos (socket no bloqueante)
        return false;
      }
      // error real
      return false;
    }
  }
}

// ====== Implementación estilo Arduino ======
int CommandParser::parseinputdata(const char* label) {
  // Usamos msg_.c_str() como "msg" de tu sketch
  const char* msg = msg_.c_str();

  if (0 == std::strcmp(label, "XY")){ // XY position
    std::sscanf(msg, "%f,%f,%f", &pX, &pY, &pZ);
    return 1;
  }
  else if (0 == std::strcmp(label, "SR")){ // Servos
    servoMode = static_cast<int>(std::strtol(msg, nullptr, 10));
    return 2;
  }
  else if (0 == std::strcmp(label, "WL")){ // Walk
    walkMode = static_cast<int>(std::strtol(msg, nullptr, 10));
    return 3;
  }
  else if (0 == std::strcmp(label, "OF")){
    std::sscanf(msg, "%d,%d", &joint, &joint_offset);
    leg_id = joint/3;
    return 4;
  }
  else if (0 == std::strcmp(label, "AG")){
    std::sscanf(msg, "%d,%d", &joint, &joint_angle);
    leg_id = joint/3;
    return 5;
  }
  else if (0 == std::strcmp(label, "LI")){
    std::sscanf(msg, "%d,%d,%d", &joint, &lowerLimit, &upperLimit);
    leg_id = joint/3;
    return 6;
  }
  else if (0 == std::strcmp(label, "CD")){
    return 7;
  }
  else if (0 == std::strcmp(label, "PI")){
    pitch = static_cast<int>(std::strtol(msg, nullptr, 10));
    return 8;
  }
  else if (0 == std::strcmp(label, "RO")){
    roll = static_cast<int>(std::strtol(msg, nullptr, 10));
    return 9;
  }
  else if (0 == std::strcmp(label, "YW")){
    yaw = static_cast<int>(std::strtol(msg, nullptr, 10));
    return 10;
  }
  else if (0 == std::strcmp(label, "TL")){
    telem = static_cast<int>(std::strtol(msg, nullptr, 10));
    return 11;
  }
  else if (0 == std::strcmp(label, "HZ")){
    hold_orient = static_cast<int>(std::strtol(msg, nullptr, 10));
    return 12;
  }
  else if (0 == std::strcmp(label, "CG")){
    cg_control = static_cast<int>(std::strtol(msg, nullptr, 10));
    return 13;
  }
  else if (0 == std::strcmp(label, "FP")){
    std::sscanf(msg, "%d,%f,%f,%f", &leg_id, &pX, &pY, &pZ);
    return 14;
  }
  else if (0 == std::strcmp(label, "CA")){
    camber = static_cast<float>(std::strtol(msg, nullptr, 10));
    return 15;
  }
  else if (0 == std::strcmp(label, "VE")){
    std::sscanf(msg, "%d,%d", &velX, &velY);
    return 16;
  }
  else if (0 == std::strcmp(label, "TR")){
    trot_gait = static_cast<int>(std::strtol(msg, nullptr, 10));
    return 17;
  }
  else if (0 == std::strcmp(label, "OS")){
    std::sscanf(msg, "%f,%f,%f", &osc_amp, &osc_w, &osc_phi);
    return 18;
  }
  else if (0 == std::strcmp(label, "OX")){
    std::sscanf(msg, "%f,%f", &oscX_amp, &oscY_amp);
    return 19;
  }
  else if (0 == std::strcmp(label, "OP")){
    std::sscanf(msg, "%f,%f", &oscX_phi, &oscY_phi);
    return 20;
  }
  else if (0 == std::strcmp(label, "RY")){
    return 21;
  }
  else if (0 == std::strcmp(label, "R")){
    return -1; // Reset
  }
  else {
    RCLCPP_INFO(logger_, "#Unknown comand");
  }

  return 0;
}
#pragma once

#include <array>
#include <cstdint>
#include <algorithm>
#include <chrono>
#include <cmath>
#include "MamdaniFuzzy.hpp"

class Horizontal_controller {
public:
    // Estado/modos
    bool hold_ON = false;
    bool cg_control = false;

    // Estimación de “suelo” (en grados)
    float floor_roll  = 0.0f;
    float floor_pitch = 0.0f;

    // Centro de gravedad (salidas) en las unidades de 'height'
    float cg_x = 0.0f;
    float cg_y = 0.0f;

    // API (misma que en tu código original)
    void init();
    void resetControllers();
    // roll/pitch en rad; height en unidades de tu cuerpo; imu_* en grados
    void newRobotOrient(float roll, float pitch, float height,
                        float imu_roll, float imu_pitch);
    // referencias y reales en grados (como en tu código original),
    // *roll y *pitch son objetivos en rad y los saturamos a ±0.87 rad
    void stabilize(float roll_ref, float pitch_ref,
                   float roll_real, float pitch_real,
                   float* roll, float* pitch);

private:
    // Controladores difusos
    MamdaniFuzzy roll_control;
    MamdaniFuzzy pitch_control;

    // Referencias (grados)
    float roll_reff  = 0.0f;
    float pitch_reff = 0.0f;

    // Buffers circulares (15)
    std::array<float, 15> pitch_ringBuffer{};
    unsigned int pitch_bufferPos = 0;

    std::array<float, 15> roll_ringBuffer{};
    unsigned int roll_bufferPos = 0;

    // Ganancias modelo/filtro
    static constexpr float roll_modelGain  = 0.8381f;
    static constexpr float pitch_modelGain = 0.8381f;
    static constexpr float filter_gain     = 0.9f;  // (no usado actualmente, pero lo conservo)

    // Muestreo
    double sampleTime_s = 0.0; // segundos
    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
};

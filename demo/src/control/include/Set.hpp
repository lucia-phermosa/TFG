#pragma once
#include <cstdint>
#include <cmath>
#include <iostream>

class Set {
    std::uint8_t type_;   // 0,1,2,3

    float p1_{0.0f};
    float p2_{0.0f};
    float p3_{0.0f};
    float p4_{0.0f};

public:
    Set() = default;

    // Configuración del tipo (0..3)
    void setType(std::uint8_t type);

    // Puntos de cada tipo
    void setPoints(float p1, float p2);                     // Type 0 & 1
    void setPoints(float p1, float p2, float p4);           // Type 2
    void setPoints(float p1, float p2, float p3, float p4); // Type 3

    // Consulta
    float getMembership(float point) const;
    float getCrispOutput() const;

    // (Opcional) acceso de solo lectura
    std::uint8_t type() const noexcept { return type_; }
};

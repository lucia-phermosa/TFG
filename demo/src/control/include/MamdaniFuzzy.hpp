#pragma once

#include <cstdint>
#include <algorithm>  // std::min
#include <iostream>   // std::cerr
#include "Set.hpp"

class MamdaniFuzzy {
public:
    // Entradas: hasta 5 conjuntos por cada una de las 2 entradas
    Set  set[2][5];
    std::uint8_t nSets[2] = {0, 0};
    float weight[2][5] = {}; // pesos (μ) tras computeMembership

    // Salida: hasta 5 conjuntos
    Set  outputSet[5];
    std::uint8_t nOutputSets {0};
    float crispOutput[5] = {}; // valores "crisp" pre-calculados por conjunto

    // Tabla de reglas (fija 3x3 como tu código original)
    // rulesTable[i][j] = índice del conjunto de salida que dispara la regla (i,j)
    int rulesTable[3][3] = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0}
    };

    // Calcula las membresías de las 2 entradas
    void computeMembership(float point1, float point2);

    // Defuzzificación Mamdani (promedio ponderado de "crispOutput" con AND = min)
    float defuzzify();
};

#include "MamdaniFuzzy.hpp"
#include <algorithm>  // std::min
#include <iostream>   // std::cerr

void MamdaniFuzzy::computeMembership(float point1, float point2) {
    // Para la entrada 0
    for (std::uint8_t i = 0; i < nSets[0] && i < 5; ++i) {
        weight[0][i] = set[0][i].getMembership(point1);
    }
    // Para la entrada 1
    for (std::uint8_t i = 0; i < nSets[1] && i < 5; ++i) {
        weight[1][i] = set[1][i].getMembership(point2);
    }
}

float MamdaniFuzzy::defuzzify() {
    // Tu tabla de reglas es 3x3: no iteramos más allá de eso
    constexpr int RULE_ROWS = 3;
    constexpr int RULE_COLS = 3;

    const int rows = std::min<int>(nSets[0], RULE_ROWS);
    const int cols = std::min<int>(nSets[1], RULE_COLS);

    float weightSum = 0.0f;
    float numerator = 0.0f;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            const int numOutSet = rulesTable[i][j];

            // Validación del índice del conjunto de salida
            if (numOutSet < 0 || numOutSet >= static_cast<int>(nOutputSets) || numOutSet >= 5) {
                std::cerr << "Warning: rulesTable[" << i << "][" << j
                          << "] = " << numOutSet
                          << " fuera de rango (nOutputSets=" << static_cast<int>(nOutputSets)
                          << "). Se ignora la regla.\n";
                continue;
            }

            // "Crisp" del conjunto de salida (puedes precomputarlo si quieres)
            crispOutput[numOutSet] = outputSet[numOutSet].getCrispOutput();

            // AND = min(μ_in1, μ_in2)
            const float wij = std::min(weight[0][i], weight[1][j]);

            numerator += crispOutput[numOutSet] * wij;
            weightSum += wij;
        }
    }

    if (weightSum == 0.0f) {
        std::cerr << "Warning: defuzzify() -> weightSum = 0 (no se activó ninguna regla)\n";
        return 0.0f; // valor por defecto para evitar NaN
    }

    return numerator / weightSum;
}

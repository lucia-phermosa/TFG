#include "Set.hpp"

void Set::setType(std::uint8_t type) {
    if (type < 4) {
        type_ = type;
    } else {
        std::cerr << "Error: The set type is not valid\n";
    }
}

void Set::setPoints(float a, float b) {
    if (type_ == 0) {
        // Tipo 0: usa p3, p4
        p3_ = a;
        p4_ = b;
    } else if (type_ == 1) {
        // Tipo 1: usa p1, p2
        p1_ = a;
        p2_ = b;
    } else {
        std::cerr << "Error: Number of arguments do not correspond to set type\n";
    }
}

void Set::setPoints(float a, float b, float d) {
    if (type_ == 2) {
        // Tipo 2: usa p1, p2, p4
        p1_ = a;
        p2_ = b;
        p4_ = d;
    } else {
        std::cerr << "Error: Number of arguments do not correspond to set type\n";
    }
}

void Set::setPoints(float a, float b, float c, float d) {
    if (type_ == 3) {
        // Tipo 3: usa p1, p2, p3, p4
        p1_ = a;
        p2_ = b;
        p3_ = c;
        p4_ = d;
    } else {
        std::cerr << "Error: Number of arguments do not correspond to set type\n";
    }
}

float Set::getMembership(float point) const {
    // Para evitar divisiones por cero
    auto safeDiv = [](float num, float den) -> float {
        return (std::fabs(den) > 1e-9f) ? (num / den) : 0.0f;
    };

    switch (type_) {
        case 0: // forma: 1 hasta p3, cae lineal a 0 en p4
            if (point <= p3_) {
                return 1.0f;
            } else if (point < p4_) {
                return safeDiv((p4_ - point), (p4_ - p3_));
            } else {
                return 0.0f;
            }

        case 1: // forma: 0 hasta p1, sube lineal a 1 en p2
            if (point <= p1_) {
                return 0.0f;
            } else if (point < p2_) {
                return safeDiv((point - p1_), (p2_ - p1_));
            } else {
                return 1.0f;
            }

        case 2: // triangular: p1 -> p2 -> p4
            if (point <= p1_) {
                return 0.0f;
            } else if (point <= p2_) {
                return safeDiv((point - p1_), (p2_ - p1_));
            } else if (point < p4_) {
                return safeDiv((p4_ - point), (p4_ - p2_));
            } else {
                return 0.0f;
            }

        case 3: // trapezoidal: p1 -> p2 (sube), p2..p3 (1), p3->p4 (baja)
            if (point <= p1_) {
                return 0.0f;
            } else if (point < p2_) {
                return safeDiv((point - p1_), (p2_ - p1_));
            } else if (point <= p3_) {
                return 1.0f;
            } else if (point < p4_) {
                return safeDiv((p4_ - point), (p4_ - p3_));
            } else {
                return 0.0f;
            }
        default:
            std::cerr << "Error: getMembership called with invalid type\n";
            return 0.0f;
    }
}

float Set::getCrispOutput() const {
    // Weighted-average “rápido” según tu implementación original
    switch (type_) {
        case 2: // triangular
            return p2_;
        case 3: // trapezoidal
            return (p2_ + p3_) * 0.5f;
        default:
            // Para type 0/1 no había definición en el original
            std::cerr << "Warning: getCrispOutput not defined for this type\n";
            return 0.0f;
    }
}

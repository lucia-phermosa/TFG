#include "Kinematics.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline float rad2deg(float r){ return r * 180.0f / float(M_PI); }

void getJoint_byPos(float x, float y, float z,
                    int l0, int l1, int l2,
                    int* q0, int* q1, int* q2)
{
    // trabajar en float, casteando longitudes int -> float
    const float l0f = static_cast<float>(l0);
    const float l1f = static_cast<float>(l1);
    const float l2f = static_cast<float>(l2);

    // utilitario local: rad -> deg (float)
    auto to_deg = [](float r) -> float {
        return r * (180.0f / static_cast<float>(M_PI));
    };

    float y_off   = y + l0f;
    float diag_sq = z*z + y_off*y_off;

    // evitar valores negativos por precisión numérica
    float under_h = std::max(0.0f, diag_sq - l0f*l0f);
    float h       = std::sqrt(under_h);

    // ángulos en radianes (todo en float)
    float gamma = static_cast<float>(M_PI)*0.5f - std::atan2(y_off, -z);
    float theta = std::atan2(h, l0f);
    float a0    = theta - gamma;

    // salida entera con redondeo
    *q0 = static_cast<int>(std::lround(static_cast<double>(to_deg(a0))));

    float beta    = std::atan2(-h, x);
    float       cos_q2  = (x*x + h*h - l1f*l1f - l2f*l2f) / (2.0f * l1f * l2f);
    cos_q2              = std::clamp(cos_q2, -1.0f, 1.0f);
    float sin_q2  = std::sqrt(std::max(0.0f, 1.0f - cos_q2*cos_q2));
    float a2      = std::atan2(sin_q2, cos_q2);

    float alpha   = std::atan2(l2f * std::sin(a2), l1f + l2f * cos_q2);
    float a1      = beta - alpha;

    *q1 = static_cast<int>(std::lround(static_cast<double>(to_deg(a1))));
    *q2 = static_cast<int>(std::lround(static_cast<double>(to_deg(a2))));
}

// ---------------------------------------------
//  IK izquierda (espejo)
// ---------------------------------------------
void getLeftJoint_byPos(float x, float y, float z,
                        int l0, int l1, int l2,
                        int* q0, int* q1, int* q2)
{
    // espejo en Y (ajusta signos según tu convención de joints)
    getJoint_byPos(x, -y, z, l0, l1, l2, q0, q1, q2);
    *q1 = -*q1;
    *q2 = -*q2;
}

// ---------------------------------------------
//  Rotación pitch (X-Z), l0 es int
// ---------------------------------------------
void computePitch(float pitch, float bodyLength, float z,
                  float* relX, float* relZ)
{
    float l = 0.5f * bodyLength; // evita división entera
    float x = l + *relX;

    float c = std::cos(pitch);
    float s = std::sin(pitch);

    float nx = x * c - z * s;
    float nz = x * s + z * c;

    *relX = l - nx;
    *relZ = nz;
}

// ---------------------------------------------
//  Rotación roll (Y-Z), l0 es int
// ---------------------------------------------
void computeRoll(float roll, float bodyWidth, float l0, float z,
                 float* relY, float* relZ)
{
    float w = 0.5f * bodyWidth + l0;
    float y = w + *relY;

    float c = std::cos(roll);
    float s = std::sin(roll);

    float ny = y * c - z * s;
    float nz = y * s + z * c;

    *relY = ny - w;
    *relZ = nz;
}

// ---------------------------------------------
//  Rotación yaw (X-Y), l0 es int
// ---------------------------------------------
void computeYaw(float yaw, float bodyWidth, float bodyLength, float l0,
                float* relX, float* relY)
{
    float l = 0.5f * bodyLength;
    float w = 0.5f * bodyWidth + l0;

    float x = *relX + l;
    float y = *relY + w;

    float c = std::cos(yaw);
    float s = std::sin(yaw);

    float ny = y * c - x * s;
    float nx = y * s + x * c;

    *relX = nx - l;
    *relY = ny - w;
}

void rotYaw(float yaw, float* x, float* y) {
  const float c = std::cos(yaw), s = std::sin(yaw);
  const float X = *x, Y = *y;
  *x =  c*X - s*Y;
  *y =  s*X + c*Y;
}

void rotPitch(float pitch, float* x, float* z) {
  const float c = std::cos(pitch), s = std::sin(pitch);
  const float X = *x, Z = *z;
  *x =  c*X - s*Z;
  *z =  s*X + c*Z;
}

void rotRoll(float roll, float* y, float* z) {
  const float c = std::cos(roll), s = std::sin(roll);
  const float Y = *y, Z = *z;
  *y =  c*Y - s*Z;
  *z =  s*Y + c*Z;
}

// ---------------------------------------------
//  Movimiento de paso; límites en float
// ---------------------------------------------
void walkMovement(int* wlk_state,
                  float* next_pX, float* next_pY, float* next_pZ,
                  float dX, float dY, float dZ)
{
    // Límites (float) para evitar comparaciones int<->float
    const float maxX =  300.0f;  // real x10
    const float minX = -300.0f;
    const float maxY =  400.0f;
    const float minY = -400.0f;
    const float maxZ = 1600.0f;
    const float minZ = 1300.0f;

    if (*wlk_state == 1) {
        *next_pZ  = maxZ;
        *next_pX -= dX;
        *next_pY -= dY;
    }
    else if (*wlk_state == 2) {
        *next_pZ  = minZ;
        *next_pX += dX;
        *next_pY += dY;
    }
    else if (*wlk_state == 12) {
        *next_pZ -= dZ;
    }
    else if (*wlk_state == 21) {
        *next_pZ += dZ;
    }

    const bool isOutOfLimits =
        (*next_pX < minX) || (*next_pX > maxX) ||
        (*next_pY < minY) || (*next_pY > maxY);

    if (*wlk_state == 1 && isOutOfLimits) {
        *wlk_state = 12;
    }
    else if (*wlk_state == 2 && isOutOfLimits) {
        *wlk_state = 21;
    }
    else if (*wlk_state == 12 && *next_pZ < minZ) {
        *wlk_state = 2;
    }
    else if (*wlk_state == 21 && *next_pZ > maxZ) {
        *wlk_state = 1;
    }
}
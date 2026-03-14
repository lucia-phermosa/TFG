#include "Kinematics.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline float clampf(float v, float lo, float hi) {
  return std::min(hi, std::max(lo, v));
}
static inline float norm3(float x, float y, float z) {
  return std::sqrt(x*x + y*y + z*z);
}

static inline float hypot2(float a, float b) {
  return std::sqrt(a*a + b*b);
}

void getJoint_byPos(rclcpp::Logger logger_,
                    float pX, float pY, float pZ,
                    const LegOffsets& off,
                    float* q0, float* q1, float* q2)
{
  if (!off.valid) {
    *q0 = *q1 = *q2 = 0.f;
    return;
  }

  // ----------------------------
  // A) Resolver q0 (hip) haciendo que el Y sea consistente
  // ----------------------------
  const float y_const  = off.t_el.y + off.t_ft.y;     // Y fijo de la cadena (no depende de q1/q2)
  const float y_target = y_const + off.t_sh.y;        // porque luego restaremos t_sh

  const float a = pY;
  const float b = -pZ;
  const float r = std::sqrt(a*a + b*b);

  float q0_raw = std::atan2(pY, -pZ); // fallback

  if (r > 1e-6f) {
    float c = clampf(y_target / r, -1.f, 1.f);
    float delta = std::acos(c);
    float alpha = std::atan2(b, a); // tal que a*cos(q)+b*sin(q)=r*cos(q-alpha)

    float qA = alpha + delta;
    float qB = alpha - delta;

    // elige la solución más cercana al fallback
    auto wrap = [](float x){
      while (x >  M_PI) x -= static_cast<float>(2*M_PI);
      while (x < -M_PI) x += static_cast<float>(2*M_PI);
      return x;
    };
    float dA = std::fabs(wrap(qA - q0_raw));
    float dB = std::fabs(wrap(qB - q0_raw));
    q0_raw = (dA < dB) ? qA : qB;
  }

  *q0 = q0_raw;

  // ----------------------------
  // B) Rotar el objetivo por -q0_raw (ojo: aquí usamos q0_raw físico, antes del signo URDF)
  //     para llevarlo al "plano" del hombro
  // ----------------------------
  const float c0 = std::cos(-q0_raw);
  const float s0 = std::sin(-q0_raw);

  const float x1 = pX;
//   const float y1 = pY*c0 - pZ*s0;
  const float z1 = pY*s0 + pZ*c0;

  // ----------------------------
  // C) Pasar al origen del hombro restando t_sh COMPLETO
  // ----------------------------
  const float xs = x1 - off.t_sh.x;
//   const float ys = y1 - off.t_sh.y;   // para debug: debería quedar ~ y_const
  const float zs = z1 - off.t_sh.z;

  // (opcional debug)
  // RCLCPP_INFO(logger_, "ys(after q0) = %.3f (should be ~ %.3f)", ys, y_const);

  // ----------------------------
  // D) IK planar XZ teniendo en cuenta que los eslabones están "diagonales" en q=0
  // ----------------------------
  const float L1 = hypot2(off.t_el.x, off.t_el.z);
  const float L2 = hypot2(off.t_ft.x, off.t_ft.z);

  if (L1 < 1e-6f || L2 < 1e-6f) {
    *q1 = *q2 = 0.f;
    return;
  }

  const float phi1 = std::atan2(off.t_el.z, off.t_el.x);
  const float phi2 = std::atan2(off.t_ft.z, off.t_ft.x);

  // Convertimos al 2-link estándar:
  // q1p = q1 + phi1
  // q2p = q2 + (phi2 - phi1)
  // X = L1*cos(q1p) + L2*cos(q1p+q2p)
  // Z = L1*sin(q1p) + L2*sin(q1p+q2p)

  float X = xs;
  float Z = zs;

  float d2 = X*X + Z*Z;
  float d  = std::sqrt(d2);
  float reach = L1 + L2;

  // saturar para que no explote el coseno
  if (d > reach) {
    float s = reach / std::max(1e-6f, d);
    X *= s; Z *= s;
    d2 = X*X + Z*Z;
  }

  float c2p = (d2 - L1*L1 - L2*L2) / (2.f * L1 * L2);
  c2p = clampf(c2p, -1.f, 1.f);
  float s2p = std::sqrt(std::max(0.f, 1.f - c2p*c2p));

  // elige "codo abajo"
  float q2p = std::atan2(s2p, c2p);
  float q1p = std::atan2(Z, X) - std::atan2(L2*std::sin(q2p), L1 + L2*std::cos(q2p));

  // recuperar q1,q2 reales
  *q1 = q1p - phi1;
  *q2 = q2p - (phi2 - phi1);
}

// ---------------------------------------------
//  Yaw: rotación en plano X-Y (eje Z)
// ---------------------------------------------
void computeYaw(float yaw, float* x, float* y)
{
  float c = std::cos(yaw);
  float s = std::sin(yaw);

  float nx = (*x) * c - (*y) * s;
  float ny = (*x) * s + (*y) * c;

  *x = nx;
  *y = ny;
}

// ---------------------------------------------
//  Pitch: rotación en plano X-Z (eje Y)
//  Convención: mano derecha, pitch positivo rota X hacia +Z
// ---------------------------------------------
void computePitch(float pitch, float* x, float* z)
{
  float c = std::cos(pitch);
  float s = std::sin(pitch);

  float nx = (*x) * c + (*z) * s;
  float nz = -(*x) * s + (*z) * c;

  *x = nx;
  *z = nz;
}

// ---------------------------------------------
//  Roll: rotación en plano Y-Z (eje X)
// ---------------------------------------------
void computeRoll(float roll, float* y, float* z)
{
  float c = std::cos(-roll);
  float s = std::sin(-roll);

  float ny = (*y) * c - (*z) * s;
  float nz = (*y) * s + (*z) * c;

  *y = ny;
  *z = nz;
}

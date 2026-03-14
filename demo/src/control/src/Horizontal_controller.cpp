#include "Horizontal_controller.hpp"

using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

void Horizontal_controller::init() {
  // --- ROLL (error)
  roll_control.nSets[0] = 3;
  roll_control.set[0][0].setType(0);
  roll_control.set[0][0].setPoints(-3, -1);
  roll_control.set[0][1].setType(3);
  roll_control.set[0][1].setPoints(-3, -1, 1, 3);
  roll_control.set[0][2].setType(1);
  roll_control.set[0][2].setPoints(1, 3);
  // Dummy variable (2ª entrada)
  roll_control.nSets[1] = 1;
  roll_control.set[1][0].setType(2);
  roll_control.set[1][0].setPoints(-1, 0, 1);
  // Outputs
  roll_control.nOutputSets = 3;
  roll_control.outputSet[0].setType(2);
  roll_control.outputSet[0].setPoints(-55, -50, -45);
  roll_control.outputSet[1].setType(2);
  roll_control.outputSet[1].setPoints(-5, 0, 5);
  roll_control.outputSet[2].setType(2);
  roll_control.outputSet[2].setPoints(45, 50, 55);
  // Rules (3x3, solo usamos la col 0 como en tu código original)
  roll_control.rulesTable[0][0] = 0;
  roll_control.rulesTable[1][0] = 1;
  roll_control.rulesTable[2][0] = 2;

  // --- PITCH (error)
  pitch_control.nSets[0] = 3;
  pitch_control.set[0][0].setType(0);
  pitch_control.set[0][0].setPoints(-3, -1);
  pitch_control.set[0][1].setType(3);
  pitch_control.set[0][1].setPoints(-3, -1, 1, 3);
  pitch_control.set[0][2].setType(1);
  pitch_control.set[0][2].setPoints(1, 3);
  // Dummy variable
  pitch_control.nSets[1] = 1;
  pitch_control.set[1][0].setType(2);
  pitch_control.set[1][0].setPoints(-1, 0, 1);
  // Outputs
  pitch_control.nOutputSets = 3;
  pitch_control.outputSet[0].setType(2);
  pitch_control.outputSet[0].setPoints(-55, -50, -34);
  pitch_control.outputSet[1].setType(2);
  pitch_control.outputSet[1].setPoints(-5, 0, 5);
  pitch_control.outputSet[2].setType(2);
  pitch_control.outputSet[2].setPoints(45, 50, 55);
  // Rules
  pitch_control.rulesTable[0][0] = 0;
  pitch_control.rulesTable[1][0] = 1;
  pitch_control.rulesTable[2][0] = 2;

  // Buffers a 0
  pitch_ringBuffer.fill(0.0f);
  roll_ringBuffer.fill(0.0f);

  // Tiempo
  last_time = steady_clock::now();
  sampleTime_s = 0.0;
}

void Horizontal_controller::resetControllers() {
  last_time = steady_clock::now();
  sampleTime_s = 0.0;
}

void Horizontal_controller::newRobotOrient(float model_roll,
                                           float model_pitch,
                                           float height,
                                           float imu_roll,
                                           float imu_pitch)
{

  if (hold_ON) {
    // referencias también en rad
    floor_roll  = roll_reff  - model_roll;
    floor_pitch = pitch_reff - model_pitch;
  } else {
    floor_roll  = imu_roll  - model_roll;
    floor_pitch = imu_pitch - model_pitch;
  }

  // CG shift (modelo geométrico)
  // pitch → x, roll → y
  cg_x = height * std::tan(floor_pitch);
  cg_y = height * std::tan(floor_roll);
}

void Horizontal_controller::stabilize(float roll_ref, float pitch_ref,
                                      float roll_real, float pitch_real,
                                      float* roll, float* pitch) 
{
  using namespace std::chrono;

  auto now = steady_clock::now();
  sampleTime_s = duration_cast<duration<double>>(now - last_time).count();
  last_time = now;

  // Errores (rad)
  float e_roll  = roll_ref  - roll_real;
  float e_pitch = pitch_ref - pitch_real;

  // Fuzzy (espera rad)
  roll_control.computeMembership(e_roll, 0.0f);
  float droll = roll_control.defuzzify();   // rad/s o rad

  pitch_control.computeMembership(e_pitch, 0.0f);
  float dpitch = pitch_control.defuzzify();

  // Integración
  roll_int  += droll  * static_cast<float>(sampleTime_s);
  pitch_int += dpitch * static_cast<float>(sampleTime_s);

  // Saturación ±50°
  constexpr float LIM = 0.87f; // rad
  roll_int  = std::clamp(roll_int,  -LIM, LIM);
  pitch_int = std::clamp(pitch_int, -LIM, LIM);

  if (roll)  *roll  = roll_int;
  if (pitch) *pitch = pitch_int;
}

void Horizontal_controller::resetPID()
{
  roll_pid_rate_.i = 0.0f;
  roll_pid_rate_.prev_e = 0.0f;
  roll_pid_rate_.d_filt = 0.0f;

  pitch_pid_rate_.i = 0.0f;
  pitch_pid_rate_.prev_e = 0.0f;
  pitch_pid_rate_.d_filt = 0.0f;

  roll_int = 0.0f;
  pitch_int = 0.0f;

  last_time = std::chrono::steady_clock::now();
}

void Horizontal_controller::stabilizePID(const rclcpp::Logger& logger_,
                                         float roll_ref, float pitch_ref,
                                         float roll_real, float pitch_real,
                                         float* roll_out, float* pitch_out)
{
  using namespace std::chrono;

  // dt robusto
  const auto now = steady_clock::now();
  float dt = duration_cast<duration<float>>(now - last_time).count();
  last_time = now;

  // clamp dt para evitar derivadas gigantes (jitter)
  dt = std::clamp(dt, 1e-3f, 0.05f);  // 1ms..50ms

  // error (rad)
  const float e_roll  = roll_ref  - roll_real;
  const float e_pitch = pitch_ref - pitch_real;

  auto stepRatePID = [&](PIDRateState& pid, float e) -> float {
    // derivada filtrada (1er orden)
    const float de = (e - pid.prev_e) / dt;
    pid.prev_e = e;

    constexpr float d_alpha = 0.85f;       // 0.8..0.95
    pid.d_filt = d_alpha * pid.d_filt + (1.0f - d_alpha) * de;

    // integral opcional (si ki=0 no hace nada)
    pid.i += e * dt;

    // rate command (rad/s)
    float u = pid.kp * e + pid.ki * pid.i + pid.kd * pid.d_filt;

    // limit rate (MUY importante)
    constexpr float MAX_RATE = 3.0f;       // rad/s (ajusta)
    u = std::clamp(u, -MAX_RATE, MAX_RATE);

    return u;
  };

  const float roll_rate  = stepRatePID(roll_pid_rate_,  e_roll);
  const float pitch_rate = stepRatePID(pitch_pid_rate_, e_pitch);

  // integración única (como fuzzy)
  roll_int  += roll_rate  * dt;
  pitch_int += pitch_rate * dt;

  // saturación final de ángulo
  constexpr float LIM = 0.87f; // ±50°
  roll_int  = std::clamp(roll_int,  -LIM, LIM);
  pitch_int = std::clamp(pitch_int, -LIM, LIM);

  if (roll_out)  *roll_out  = roll_sign_  * roll_int;
  if (pitch_out) *pitch_out = pitch_sign_  * pitch_int;

  // Debug útil si quieres:
  // RCLCPP_INFO(logger_, "[PID] e_roll=%.3f rate=%.3f roll_int=%.3f | e_pitch=%.3f rate=%.3f pitch_int=%.3f",
  //             e_roll, roll_rate, roll_int, e_pitch, pitch_rate, pitch_int);
}

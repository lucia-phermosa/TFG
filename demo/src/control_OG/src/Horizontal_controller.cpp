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

void Horizontal_controller::newRobotOrient(float roll, float pitch, float height,
                                           float imu_roll, float imu_pitch) {
    // roll/pitch (modelo) vienen en rad → aplicar ganancia y meter al buffer
    roll_ringBuffer[roll_bufferPos % 15] = roll * roll_modelGain;
    roll_bufferPos++;
    float modelRoll = roll_ringBuffer[roll_bufferPos % 15]; // misma lógica que tu original (lee el siguiente slot)

    pitch_ringBuffer[pitch_bufferPos % 15] = pitch * pitch_modelGain;
    pitch_bufferPos++;
    float modelPitch = pitch_ringBuffer[pitch_bufferPos % 15];

    // IMU ya viene en grados (como en tu código original)
    float floor_x = 0.0f, floor_y = 0.0f;
    if (hold_ON) {
        // referencias y modelos en grados
        floor_x = roll_reff  - (modelRoll  * 180.0f / static_cast<float>(M_PI));
        floor_y = pitch_reff - (modelPitch * 180.0f / static_cast<float>(M_PI));
    } else {
        // IMU en grados menos modelos en grados
        floor_x = imu_roll  - (modelRoll  * 180.0f / static_cast<float>(M_PI));
        floor_y = imu_pitch - (modelPitch * 180.0f / static_cast<float>(M_PI));
    }

    // Si quieres filtrar:
    // floor_roll  = floor_roll  * filter_gain + floor_x * (1 - filter_gain);
    // floor_pitch = floor_pitch * filter_gain + floor_y * (1 - filter_gain);
    // De momento, directo (como dejaste en tu último código):
    floor_roll  = floor_x;
    floor_pitch = floor_y;

    // CG en unidades de 'height'
    // cg_x usa tan(pitch_floor), cg_y usa tan(roll_floor). Ambos ángulos en rad.
    cg_x = static_cast<float>(height * std::tan(floor_pitch * static_cast<float>(M_PI) / 180.0f));
    cg_y = static_cast<float>(height * std::tan(floor_roll  * static_cast<float>(M_PI) / 180.0f));
}

void Horizontal_controller::stabilize(float roll_ref, float pitch_ref,
                                      float roll_real, float pitch_real,
                                      float* roll, float* pitch) {
    // Tiempo de muestreo (segundos)
    const auto now = steady_clock::now();
    sampleTime_s = duration_cast<duration<double>>(now - last_time).count();
    last_time = now;

    // Guardar referencias (grados)
    roll_reff  = roll_ref;
    pitch_reff = pitch_ref;

    // ROLL: entrada = error (grados), 2ª entrada dummy = 0
    roll_control.computeMembership(roll_ref - roll_real, 0.0f);
    float droll_deg = roll_control.defuzzify();     // deg
    // Δroll (rad) = deg → rad, escalado por Ts
    float droll_rad = droll_deg * static_cast<float>(M_PI / 180.0) * static_cast<float>(sampleTime_s);
    if (roll) {
        *roll += droll_rad;
        // Saturación ±0.87 rad
        *roll = std::clamp(*roll, -0.87f, 0.87f);
    }

    // PITCH
    pitch_control.computeMembership(pitch_ref - pitch_real, 0.0f);
    float dpitch_deg = pitch_control.defuzzify();   // deg
    float dpitch_rad = dpitch_deg * static_cast<float>(M_PI / 180.0) * static_cast<float>(sampleTime_s);
    if (pitch) {
        *pitch += dpitch_rad;
        *pitch = std::clamp(*pitch, -0.87f, 0.87f);
    }
}

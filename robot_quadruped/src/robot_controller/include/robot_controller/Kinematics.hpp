#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <cmath>

void getJoint_byPos(float x, float y, float z, float l0, float l1, float l2, int* q0, int* q1, int* q2);

void getLeftJoint_byPos(float x, float y, float z, float l0, float l1, float l2, int* q0, int* q1, int* q2);

void computePitch(float pitch, float bodyLength, float z, float* relX, float* relZ);

void computeRoll(float roll, float bodyWidth, float l0, float z, float* relY, float* relZ);

void computeYaw(float yaw, float bodyWidth, float bodyLength, float l0, float* relX, float* relY);

void walkMovement(int* wlk_state, int* next_pX, int* next_pY, int* next_pZ, int dX, int dY, int dZ);

#endif
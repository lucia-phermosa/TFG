#pragma once
#include <cmath>
#include <algorithm> 

// Right leg inverse kinematics (as in your Arduino code)
void getJoint_byPos(float x, float y, float z, int l0, int l1, int l2, int* q0, int* q1, int* q2);

// Left leg IK (mirror around sagittal plane). If your original has a different convention, adjust.
void getLeftJoint_byPos(float x, float y, float z, int l0, int l1, int l2, int* q0, int* q1, int* q2);

void computePitch(float pitch, float bodyLength, float z, float* relX, float* relZ);
void computeRoll(float roll, float bodyWidth, float l0, float z, float* relY, float* relZ);
void computeYaw(float yaw, float bodyWidth, float bodyLength, float l0, float* relX, float* relY);

void rotYaw(float yaw, float* x, float* y);
void rotPitch(float pitch, float* x, float* z);
void rotRoll(float roll, float* y, float* z);

void walkMovement(int* wlk_state, float* next_pX, float* next_pY, float* next_pZ, float dX, float dY, float dZ);
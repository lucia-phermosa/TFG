#pragma once
#include <cmath>
#include <algorithm> 
#include <rclcpp/rclcpp.hpp>
#include "Enums.hpp"

void getJoint_byPos(rclcpp::Logger logger_,
                    float pX, float pY, float pZ,
                    const LegOffsets& off,
                    float* q0, float* q1, float* q2);

void computePitch(float pitch, float* x, float* z);
void computeRoll(float roll, float* y, float* z);
void computeYaw(float yaw, float* x, float* y);

#ifndef LEG_HPP
#define LEG_HPP

#include "robot_controller/Kinematics.hpp"
#include "robot_controller/Planner.hpp"

enum Side{left=-1, right=1};
enum End{front=-1, rear=1};
enum Joint{sh1, sh2, elbow};

#ifndef MOVEMENT_TYPE
#define MOVEMENT_TYPE
enum MovementType {absolute, incremental};
#endif

class Leg {
    Side side;
    End end;

    int q0, q1, q2;
    int l0, l1, l2;

    int ang1, ang2, ang3; //Servos angle

    float foot_speed = 0.4; // mm/ms
    FootPlanner footPlanner;
    
    //Relative position of the shoulder from body's center
    int shoulderPosX;
    int shoulderPosY;

    //Timers
    unsigned long now;
    unsigned long last = 0;
    unsigned long dt = 5;  //ms between trayectory points (min 5)

public:
    
    int next_pX = 0, next_pY = 0, next_pZ;
    int wlk_state = 1;

    int footX = 0, footY = 0, footZ = 0;
    int q0_offset, q1_offset, q2_offset;

    Leg();
    Leg(int length_l0, int length_l1, int length_l2, Side side, End end);
    void set_joint_offset(int joint, int offset);
    void initAbsFootPos(int bodyLength, int bodyWidth);
    void update(int pX, int pY, int pZ);
    void go2pos(int px, int py, int pz, int dz, int type = absolute);
    bool arrived2pos();
    //void walk(int velX, int velY, int velZ);
    void resetWalk();
    void setJointsAngle(int ang_h1, int ang_h2, int ang_sh, bool limits=true);
    void setServoAngle(int joint, int angle);
    void setServosAngle(int ang1, int ang2, int ang3);
    void sendCalibData();
    void setFootPos(int px, int py, int pz);
    void getFoot2ReffVect(int bodyX, int bodyY, int *distX, int *distY);
};

#endif
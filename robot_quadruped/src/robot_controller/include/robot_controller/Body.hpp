#ifndef BODY_HPP
#define BODY_HPP

#include "robot_controller/Leg.hpp"
#include "robot_controller/Planner.hpp"

enum LegID {FR, RR, FL, RL};

#ifndef MOVEMENT_TYPE
#define MOVEMENT_TYPE
enum MovementType {absolute, incremental};
#endif

class Body{
    int centerPosX = 0;
    int centerPosY = 0;
    bool resetFootState = 0;
    
public:
    int posX = 0, posY = 0, posZ = 0; //in millimeters
    float roll = 0, pitch = 0, yaw = 0;
    int camber = 0;

    Leg leg[4];

    const int body_width = 90;
    const int body_length = 184;

    const float body_speed = 0.08; // mm/ms
    TrayectoryPlanner balancePlanner;
    int movemetState = 0;
    int centeringState = 0;
    bool isIdle = true;

    Body();
    void setCamber(int camber_in_mm);
    void move(int dx, int dy, int dz);
    void go2pos(int px, int py, int pz, int type = absolute);
    bool arrived2pos();
    void updatePose();
    void getMeanFootPos(int *x, int *y, int *z);
    bool moveFoot(int legID, int px, int py, int pz, int dz, int movementType = absolute, bool return2zero = true);
    int getAbsFootPos(int legID);
    bool centerPos(int dx=0, int dy=0, int dz=0);
};

#endif
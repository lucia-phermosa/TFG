#ifndef PLANNER_HPP
#define PLANNER_HPP

class TrayectoryPlanner{
    float velX, velY, velZ;
    unsigned long startTime;
public:
    int targetTime;     //in ms
    int startX, startY, startZ;
    int targetX, targetY, targetZ;
    int newX, newY, newZ;
    
    void setPlanner(float movementSpeed, int startPx, int startPy, int startPz, int finalX, int finalY, int finalZ);
    void restartTimer();
    bool computeNewPos();
};

class FootPlanner{
    
    float speed; //in mm/ms
    int targetTime;
    int state;    
public:
    TrayectoryPlanner planner;
    int px, py, pz;
    int tx, ty, tz;
    int dZ;
    int newX, newY, newZ;

    FootPlanner();
    void setPlanner(float movementSpeed, int px, int py, int pz, int tx, int ty, int tz, int dz);
    bool updatePos();
};

#endif
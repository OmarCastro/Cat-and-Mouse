#ifndef TACTICS_H
#define TACTICS_H

#include "../libRobSock/RobSock.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


//main robot state
const int RUN  = 1;
const int STOP = 2;
const int WAIT = 3;
const int RETURN = 4;
const int FINISHED = 5;


//RUN sub-states
const int BYPASSING_LEFT  = 1; // bypassing an obstacle on rigth side
const int BYPASSING_RIGTH = 2; // bypassing an obstacle on left side
const int RUNNING = 3; // isnt bypassing obstacles, runs forward looking at the beacon


struct externalRobot{
public:
    bool isCat;
    double x,y;
    externalRobot():isCat(false),x(0),y(0){}
};

struct relativePosition{
public:
    double x,y,distance;
    void update(const externalRobot& robot){
        x = GetX() - robot.x;
        y = GetY() - robot.y;
        distance = sqrt(x*x+y*y);
    }
};
extern int rob_id;

/* Calculate the power of left and right motors */
void DetermineAction(int beaconToFollow, double *lPow, double *rPow, int *state);
void DetermineMouseAction(int beaconToFollow, double *lPow, double *rPow, int *state, externalRobot *robots);



#endif // TACTICS_H

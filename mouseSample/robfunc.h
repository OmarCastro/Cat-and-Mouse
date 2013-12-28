#ifndef __ROBFUNC_H__
#define __ROBFUNC_H__

#include "RobSock.h"

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


void DetermineAction(int beaconToFollow, float *lPow, float *rPow, int *state);


#endif

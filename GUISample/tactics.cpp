#ifndef TACTICS_CPP
#define TACTICS_CPP
#include "tactics.h"

int rob_id=1;


RobotMap::RobotMap(){
    for(int i=0;i < mapsize;i++)
        for(int j=0;j < mapsize;j++)
            values[i][j]=defval;

}

const int side_front = 0;
const int side_frontleft = 1;
const int side_left = 2;
const int side_backleft = 3;
const int side_back = 4;
const int side_backright = 5;
const int side_right = 6;
const int side_frontright = 7;

/* Calculate the power of left and right motors */
void DetermineAction(int beaconToFollow, double *lPow, double *rPow, int *state)
{
    static int counter=0;
    static float CollisionOrientation = 0.0;

    bool   beaconReady;
    static struct beaconMeasure beacon; // beacon sensor
    static float  left; //value of frontal left sonar sensor
    static float right; //value of frontal rigth sonar sensor
    static float center; //value of frontal center sonar sensor
    //static int    Ground;
    static bool   Collision;// collision sensor
    static float Compass; //compass sensor




  // SENSORS ACCESS

    /*Access to values from Sensors - Only ReadSensors() gets new values */
    if(IsObstacleReady(LEFT))
        left=     GetObstacleSensor(LEFT);
    if(IsObstacleReady(RIGHT))
        right=    GetObstacleSensor(RIGHT);
    if(IsObstacleReady(CENTER))
        center=   GetObstacleSensor(CENTER);

    beaconReady = IsBeaconReady(beaconToFollow);
    if(beaconReady) {
       beacon =  GetBeaconSensor(beaconToFollow);
    }
    else beaconReady=0;

    //if(IsGroundReady())
    //    Ground=    GetGroundSensor();
    if(IsBumperReady())
        Collision= GetBumperSensor();
    if(IsCompassReady()){
        Compass= GetCompassSensor();
    }








    if(beaconReady && beacon.beaconVisible && center < 3.0){
            if(beacon.beaconDir > 20.0 && left < 4.0){
                *lPow=0.0;
                *rPow=0.1;
                *state = RUNNING;
            }
            else if(beacon.beaconDir < -20.0 && right < 4.0){
                *lPow=0.1;
                *rPow=0.0;
                *state = RUNNING;
            }
            else { /* Full Speed Ahead */
               *lPow=0.1;
               *rPow=0.1;
            }


    } else if(center>3.0 || right> 4.0 || left>4.0 || Collision) { /* Close Obstacle - Rotate */
        if(right < left) {
              *lPow=0.06;
               *rPow=-0.06;
            if(*state != BYPASSING_RIGTH){
                CollisionOrientation = Compass;
               *state = BYPASSING_RIGTH;
            }
        }  else {
               *lPow=-0.06;
               *rPow=0.06;
            if(*state != BYPASSING_LEFT){
                CollisionOrientation = Compass;
               *state = BYPASSING_LEFT;
            }
        }

    } else if(*state == BYPASSING_RIGTH && left>3.0){ // if its still bypassing an obstacle through the right side
        if(left < 3.5 ){
            *lPow=0.05;
            *rPow=0.07;
        } else if(left > 3.7 ){
            *lPow=0.07;
            *rPow=0.05;
        } else {
            *lPow=0.05;
            *rPow=0.05;
        }
    } else if(*state == BYPASSING_LEFT && right>3.0){ // if its still bypassing an obstacle through the left side
        if(right < 3.5 ){
            *lPow=0.07;
            *rPow=0.05;
        } else if(right > 3.7 ){
            *lPow=0.05;
            *rPow=0.07;
        } else {
            *lPow=0.05;
            *rPow=0.05;
        }
    } else {
        if(*state != RUNNING){
            //difference between orientation when approached an obstacle and current one
            float diff = Compass - CollisionOrientation;
            if(diff < 10 && diff > -10){
                *state = RUNNING;
            }
        }

        if(*state == BYPASSING_LEFT){
            *lPow=0.07;
            *rPow=0.01;

        }
        else if(*state == BYPASSING_RIGTH){
            *lPow=0.01;
            *rPow=0.07;
        }
        else { /* Full Speed Ahead */
           *lPow=0.1;
           *rPow=0.1;
        }
    }




    counter++;
}


/*
 * Negamax implementation
 *
 *   Scoring Function
 *
 *   m(x,y) = 19 * w(x,y) + sqrt(d(x,y)^3) + f(d(x,y)) / d(x,y)
 *
 *   where w(x,y) is the weight function given by
 *
 *   w(x,y) = x * (12 - x) + y * (12 - y)
 *
 *   d(x,y) is the distance from a point (x,y) and the position of the cat (xc, yc) given by
 *
 *   d(x,y) = sqrt( (x - xc)^2 + (y - yc)^2 )
 *
 *   and f(x) is a modifier function given by
 *
 *   f(x) = (x >= 9) ? 300 : (x < 3) ? -1200 : (x < 4) ? -1000 : (x < 6) ? -400 : -200
 *
 *
 */

double f(double x){
    return (x >= 6) ? 1000 : (x < 3) ? 0 : (x < 4) ? 200 : (x < 5) ? 400 : 700;
}

double negamax(double posX, double posY, double catX, double catY){
    double relX = posX - catX,
           relY = posY - catY;
    //d(x,y)
    double distance2 = relX*relX+relY*relY;
    double distance = sqrt(distance2);
    return sqrt(distance2*distance) + f(distance)/distance;

}



/* Calculate the power of left and right motors */
void DetermineMouseAction(double *lPow, double *rPow, int *state, RobotMap *map,externalRobot *robots)
{
    static int counter=0;


    static float  left; //value of frontal left sonar sensor
    static float right; //value of frontal rigth sonar sensor
    static float center; //value of frontal center sonar sensor
    static bool   Collision;// collision sensor
    static float Compass = 0; //compass sensor
    static float X = -1; // GPS x value
    static float Y = -1; //GPS yvalue


  // SENSORS ACCESS

    /*Access to values from Sensors - Only ReadSensors() gets new values */
    if(IsObstacleReady(LEFT))
        left=     GetObstacleSensor(LEFT);
    if(IsObstacleReady(RIGHT))
        right=    GetObstacleSensor(RIGHT);
    if(IsObstacleReady(CENTER))
        center=   GetObstacleSensor(CENTER);

    if(IsBumperReady())
        Collision= GetBumperSensor();
    if(IsCompassReady()){
        Compass= GetCompassSensor();
        //printf("orientation %f\n", Compass);
    }
    if(IsGPSReady()){
        if(X < 0 || Y < 0){
            map->setOffset(GetX(),GetY());
        }
        X= GetX();
        Y= GetY();
    }



    float multiplier[8] = {1,1,1,1,1,1,1,1};


    if(center>3.0 || right> 4.0 || left>4.0 || Collision) { /* Close Obstacle - Rotate */
        if(right < left) {
              *lPow=0.06;
               *rPow=-0.06;
        }  else {
               *lPow=-0.06;
               *rPow=0.06;
        }

    } else {
        if(left > 1.0){
            //give a penalty on score if theres an obstacle on the left side
            multiplier[side_frontleft] = multiplier[side_left] = multiplier[side_backleft] = 1/left;
        }
        if(right > 1.0){
            //give a penalty on score if theres an obstacle on the right side
            multiplier[side_frontright] = multiplier[side_right] = multiplier[side_backright] = 1/right;
        }
        if(center > 1.0){
            //give a penalty on score if theres an obstacle on the front
            multiplier[side_frontright] = multiplier[side_frontleft] = multiplier[side_front] = 1.25/right;
        }

        //always favor going to front to avoid slowing down
        multiplier[side_frontright] *= 1.25;
        multiplier[side_frontleft] *= 1.25;
        multiplier[side_front] *= 1.25;





        float orientations[8] = {Compass,Compass + 45,Compass + 90,Compass + 135,Compass + 180,
                Compass - 135,Compass - 90,Compass - 45};

        float distance = 2;

        float score[8];

        float maxScore = 0;
        int maxScoreSide = 0;


        for(int i=0;i<8;++i){
            float x= X + distance * cos(M_PI * orientations[i] / 180);
            float y= Y + distance * sin(M_PI * orientations[i] / 180);

            if(i == side_front){
                double val = map->getValueGPS(x,y);
                if(center < 2.0){
                    val += 0.2;
                    map->setValueGPS(x,y,(val > 1.0)?1.0:val);
                } else {
                    val -= 0.2;
                    map->setValueGPS(x,y,(val < 0.0)?0.0:val);
                }
            } else if(i == side_frontleft){
                double val = map->getValueGPS(x,y);
                if(left < 2.0){
                    val += 0.2;
                    map->setValueGPS(x,y,(val > 1.0)?1.0:val);
                } else {
                    val -= 0.2;
                    map->setValueGPS(x,y,(val < 0.0)?0.0:val);
                }
            }else if(i == side_frontright){
                double val = map->getValueGPS(x,y);
                if(right < 2.0){
                    val += 0.2;
                    map->setValueGPS(x,y,(val > 1.0)?1.0:val);
                } else {
                    val -= 0.2;
                    map->setValueGPS(x,y,(val < 0.0)?0.0:val);
                }
            }

            float points = 0;
            for(int j=0;j<5;++j){
                if(robots[j].isCat){
                    points += negamax(x,y,robots[j].x,robots[j].y);
                }
            }

            //avoid aproaching obstacles
            for(int wi=-4;wi<=4;++wi){
                for(int wj=-4;wj<=4;++wj){
                    if(map->getValueGPS(x+wi,y+wj) < 0.4){ //avoid wall
                        multiplier[i] -= 0.1;
                    }
                }
            }

            points *= multiplier[i];
            score[i] = points;
            if(i==0){
               maxScore = points;

            } else if(points > maxScore){
                maxScore = points;
                maxScoreSide = i;
            }
        }
        switch (maxScoreSide) {
        case side_front:
            *lPow=0.1;
            *rPow=0.1;
            break;
        case side_frontleft:
            *lPow=0.08;
            *rPow=0.1;
            break;
        case side_left:
            *lPow=0.03;
            *rPow=0.1;
            break;
        case side_backleft:
            *lPow=-0.02;
            *rPow=0.1;
            break;
        case side_back:
            if(score[3] > score[5]){
                *lPow=-0.1;
                *rPow=0.1;
            } else {
                *lPow=0.1;
                *rPow=-0.1;
            }
            break;
        case side_backright:
            *rPow=-0.02;
            *lPow=0.1;
            break;
        case side_right:
            *rPow=0.03;
            *lPow=0.1;
            break;
        case side_frontright:
            *rPow=0.08;
            *lPow=0.1;
            break;
        default:
            break;
        }
    }




    counter++;
}



#endif // TACTICS_CPP

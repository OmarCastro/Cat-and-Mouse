#include "robfunc.h"
#include <math.h>

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
    return (x >= 5) ? 300 : (x < 2.5) ? -1200 : (x < 3) ? -1000 : (x < 4) ? -400 : -200;
}

double negamax(double posX, double posY, double catX, double catY){
    double relX = posX - catX,
           relY = posY - catY;
    //d(x,y)
    double distance2 = relX*relX+relY*relY;
    double distance = sqrt(distance);
    return sqrt(distance2*distance) + f(distance)/distance;

}



/* Calculate the power of left and right motors */
void DetermineMouseAction(int beaconToFollow, float *lPow, float *rPow, int *state)
{
    static int counter=0;
    static float CollisionOrientation = 0.0;

    bool   beaconReady;
    static struct beaconMeasure beacon; // beacon sensor
    static float  left; //value of frontal left sonar sensor
    static float right; //value of frontal rigth sonar sensor
    static float center; //value of frontal center sonar sensor
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


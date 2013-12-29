#include "robfunc.h"
#include <stdio.h>

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
    double distance = sqrt(distance2);
    return sqrt(distance2*distance) + f(distance)/distance;

}



/* Calculate the power of left and right motors */
void DetermineMouseAction(int beaconToFollow, float *lPow, float *rPow, int *state, externalRobot *robots)
{
    static int counter=0;

    bool   beaconReady;
    static struct beaconMeasure beacon; // beacon sensor
    static float  left; //value of frontal left sonar sensor
    static float right; //value of frontal rigth sonar sensor
    static float center; //value of frontal center sonar sensor
    static bool   Collision;// collision sensor
    static float Compass = 0; //compass sensor
    static float X; // GPS x value
    static float Y; //GPS yvalue


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
        printf("orientation %f\n", Compass);
    }
    if(IsGPSReady()){
        X= GetX();
        Y= GetY();
    }






    if(center>3.0 || right> 4.0 || left>4.0 || Collision) { /* Close Obstacle - Rotate */
        if(right < left) {
              *lPow=0.06;
               *rPow=-0.06;
        }  else {
               *lPow=-0.06;
               *rPow=0.06;
        }

    } else if(left> 3.7){ // if its still bypassing an obstacle through the right side
        *lPow=0.07;
        *rPow=0.05;
    } else if(right>3.7){ // if its still bypassing an obstacle through the left side
        *lPow=0.05;
        *rPow=0.07;
    } else {



        float orientations[8] = {Compass,Compass + 45,Compass + 90,Compass + 135,Compass + 180,
                Compass - 135,Compass - 90,Compass - 45};

        float distance = 2;

        float score[8];

        float maxScore;
        int maxScoreIndex = 0;


        for(int i=0;i<8;++i){
            float x= X + distance * cos(M_PI * orientations[i] / 180);
            float y= Y + distance * sin(M_PI * orientations[i] / 180);
            float points = 0;
            for(int j=0;j<5;++j){
                if(robots[j].isCat){
                    points += negamax(x,y,robots[j].x,robots[j].y);
                }
            }
            score[i] = points;
            if(i==0){
               maxScore = points;
            } else if(points > maxScore){
                maxScore = points;
                maxScoreIndex = i;
            }
        }

        if(maxScoreIndex == 0){ //orientation front
            //go front at full speed
            *lPow=0.1;
            *rPow=0.1;

        }else if(maxScoreIndex == 1){ //orientation front left
            *lPow=0.08;
            *rPow=0.1;

        }else if(maxScoreIndex == 2){ //orientation left
            *lPow=0.03;
            *rPow=0.1;

        }else if(maxScoreIndex == 3){ //orientation back left
            *lPow=-0.02;
            *rPow=0.1;

        }else if(maxScoreIndex == 4){ //orientation back
            if(score[3] > score[5]){
                *lPow=-0.1;
                *rPow=0.1;
            } else {
                *lPow=0.1;
                *rPow=-0.1;
            }
        }else if(maxScoreIndex == 5){ //orientation back right
            *rPow=-0.02;
            *lPow=0.1;

        }else if(maxScoreIndex == 6){ //orientation right
            *rPow=0.03;
            *lPow=0.1;

        }else if(maxScoreIndex == 7){ //orientation front right
            *rPow=0.08;
            *lPow=0.1;

        }


    }




    counter++;
}


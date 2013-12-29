/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2011 Universidade de Aveiro

    ciberRatoToolsSrc is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    ciberRatoToolsSrc is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* main.cpp
 *
 * Basic Robot Agent
 * simple version for demonstration
 *
 * For more information about the CiberRato Robot Simulator 
 * please see http://microrato.ua.pt/ or contact us.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <iostream>

using std::cerr;

#include <qapplication.h>
#include "../libRobSock/RobSock.h"
#include "sampapp.h"
#include "robview.h"




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
int rob_id=1;
int actionState = RUNNING;
    externalRobot robots[5];

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
void DetermineMouseAction(int beaconToFollow, double *lPow, double *rPow, int *state, externalRobot *robots)
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



    float multiplier[8] = {1,1,1,1,1,1,1,1};


    if(center>3.0 || right> 4.0 || left>4.0 || Collision) { /* Close Obstacle - Rotate */
        if(right < left) {
              *lPow=0.06;
               *rPow=-0.06;
        }  else {
               *lPow=-0.06;
               *rPow=0.06;
        }

    } else if(left> 3.7){ // if its still bypassing an obstacle through the right side
        multiplier[1] = 0;
        multiplier[2] = 0;
        multiplier[3] = 0;
    } else if(right>3.7){ // if its still bypassing an obstacle through the left side
        multiplier[5] = 0;
        multiplier[6] = 0;
        multiplier[7] = 0;
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
            points *= multiplier[i];
            score[i] = points;
            if(i==0){
               maxScore = points;

            } else if(points > maxScore){
                maxScore = points;
                maxScoreIndex = i;
            }
        }
        printf("maxScore: %d\n", maxScore);
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


/** SampApp methods **/

SampApp::SampApp(int &argc, char*argv[], char *robot_name, int robot_type) : QApplication(argc,argv)
{
    strncpy(rob_name, robot_name, 19);
    rob_name[19]='\0';
    rob_type = robot_type;
    beaconToFollow = 0; // start by finding target at beacon 0
}


void SampApp::act(void)
{
     static int state=STOP,stoppedState=RUN;
     double lPow, rPow;

     /* Reading next values from Sensors */
     ReadSensors();

     if(GetFinished()) /* Simulator has received Finish() or Robot Removed */
     {
        printf(  "%s Exiting\n", rob_name );
        exit(0);
     }
     if(state==STOP && GetStartButton()) state=stoppedState;  /* Restart     */
     if(state!=STOP && GetStopButton())  {
         stoppedState=state;
         state=STOP; /* Interrupt */
     }


     if(rob_type == 0){  //if its a cat he is conditioned to tell its GPS position
         if(IsGPSReady()){
             char msg[100];
             sprintf(msg, "p %f %f",GetX(),GetY());
             printf("\n CAT %i IS ON %f %f \n", rob_id, GetX(),GetY());
             Say(msg);
         }



     switch (state) {
              case RUN:    /* Go */
       if( GetVisitingLed() ) state = WAIT;
               DetermineAction(0,&lPow,&rPow,&actionState);
               DriveMotors(lPow,rPow);
               break;
      case WAIT: /* Wait for others to visit target */
          if(GetReturningLed()) state = RETURN;

                  DriveMotors(0.0,0.0);
                  break;
      case RETURN: /* Return to home area */
          if(GetGroundSensor()==1) { /* Finish */
                      Finish();
                      printf("%s found home at %d\n", rob_name, GetTime());
                  }
                  else {
                     Finish();
                     //DetermineAction(1,&lPow,&rPow,&actionState);
                     //DriveMotors(lPow,rPow);
                  }
                  break;
         }

         for(int i=0;i<5;i++){
             int id = i+1;
             if(rob_id == id) continue;
             if(NewMessageFrom(id)){
                 char* msg = GetMessageFrom(id);
                 if(msg[0] == 'm'){ //check if the message is about catching mouse
                     int id_mouse, id_cat;
                     sscanf(msg+2, "%d %d",&id_mouse,&id_cat);
                     if(rob_id == id_cat){
                         Finish();
                     }
                 }
             }
         }
     } else {
         for(int i=0;i<5;i++){
             int id = i+1;
             if(rob_id == id) continue;
             if(NewMessageFrom(id)){
                 char* msg = GetMessageFrom(id);
                 if(msg[0] == 'p'){ //check if the message is about position
                     sscanf(msg+2, "%lf %lf",&robots[i].x,&robots[i].y);
                     robots[i].isCat = true;
                 }
             }

             if(IsGPSReady() && robots[i].isCat){
                 relativePosition pos;
                 pos.update(robots[i]);
                 printf("cat %d at relative position %f %f, distance: %f\n",id, pos.x ,pos.y,pos.distance);
                 if(pos.distance < 0.5){
                     char msg[10];
                     sprintf(msg, "m %d %d",rob_id,id);
                     Say(msg);
                     printf("mouse %d is dead", rob_id);
                     Finish();
                 }
             }

         }

                 DetermineMouseAction(0,&lPow,&rPow,&actionState,robots);
                 DriveMotors(lPow,rPow);





     }

  //Request Sensors for next cycle
   if(GetTime() % 2 == 0) {
         RequestObstacleSensor(CENTER);

         //RequestCompassSensor();

         if(GetTime() % 4 == 0 || beaconToFollow == GetNumberOfBeacons())
             RequestGroundSensor();
         else
             RequestBeaconSensor(beaconToFollow);

      }
      else {
       RequestSensors(4, "IRSensor1", "IRSensor2", "Compass", "GPS");
      }
}


int main( int argc, char** argv )
{
    char host[100]="localhost";
    char rob_name[20]="GUISample";
    int type;

    printf(" GUISample Robot \n Copyright 2002-2011 Universidade de Aveiro\n");
    fflush(stdout);

    /* processing arguments */
    while (argc > 2) // every option has a value, thus argc must be 1, 3, 5, ...
    {
        if (strcmp(argv[1], "-host") == 0)
        {
           strncpy(host, argv[2], 99);
           host[99]='\0';
        }
        else if (strcmp(argv[1], "-robname") == 0)
        {
           strncpy(rob_name, argv[2], 19);
           rob_name[19]='\0';
        }
        else if (strcmp(argv[1], "-pos") == 0)
        {
            if(sscanf(argv[2], "%d", &rob_id)!=1)
               argc=0; // error message will be printed
        }
        else if (strcmp(argv[1], "-type") == 0)
        {
            char typeStr[10]="cat";
           strncpy(typeStr, argv[2], 9);
           typeStr[9]='\0';
           type = (strcmp(typeStr,"mouse") == 0) ? 1:0;
        }
        else
        {
                break; // the while
        }
        argc -= 2;
        argv += 2;
    }

    if (argc != 1)
    {
        fprintf(stderr, "Bad number of parameters\n"
                "SYNOPSIS: GUISample [-host hostname] [-robname robotname] [-pos posnumber]\n");
        return 1;
    }

    // Create Qt application - Must be before InitRobot
    SampApp app( argc, argv, rob_name, type );
    qApp->addLibraryPath("../libRobSock");

    /* Connect Robot to simulator */
    double irSensorAngles[4] = { 0.0, 60.0, -60.0, 180.0};
    if(InitRobot2(rob_name,rob_id,irSensorAngles,host,type)==-1) {
          printf("%s Failed to connect\n",rob_name);
          exit(1);
    }
    printf("%s Connected\n",rob_name);
    fflush(stdout);

    // Connect event NewMessage to handler act()
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), &app, SLOT(act()));
    
    // create robot display widget
    RobView robView(irSensorAngles, rob_name);

    // Connect event NewMessage to handler redrawRobot()
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), &robView, SLOT(redrawRobot()));
    
    robView.show();

    // process events
    return app.exec();
}

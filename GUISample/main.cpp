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


#include <string.h>

#include <iostream>

using std::cerr;

#include <qapplication.h>
#include "sampapp.h"
#include "robview.h"
#include "tactics.h"

int actionState = RUNNING;

externalRobot robots[5];


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
             //printf("\n CAT %i IS ON %f %f \n", rob_id, GetX(),GetY());
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
                     beaconToFollow++;
                     if(rob_id == id_cat || beaconToFollow >= GetNumberOfBeacons()){
                         printf("bcns: %d %d\n", beaconToFollow, GetNumberOfBeacons() );
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

                 //printf("cat %d at relative position %f %f, distance: %f\n",id, pos.x ,pos.y,pos.distance);


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

         RequestCompassSensor();

         if(GetTime() % 4 != 0 && rob_type == 0 && beaconToFollow < GetNumberOfBeacons()){
             RequestBeaconSensor(beaconToFollow);
         }

      }
      else {
       RequestSensors(3, "IRSensor1", "IRSensor2", "GPS");
      }
}


int main( int argc, char** argv )
{
    char host[100]="localhost";
    char rob_name[20]="GUISample";
    int type = 0;

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
    double irSensorAngles[5] = { 0.0, 60.0, -60.0, 180.0, 0};

    if(type == 0){
        if(InitRobot2(rob_name,rob_id,irSensorAngles,host,type)==-1) {
              printf("%s Failed to connect\n",rob_name);
              exit(1);
        }
    } else {
        if(InitRobotBeacon(rob_name,rob_id,4.0,host,type)==-1){
            printf("%s Failed to connect\n",rob_name);
            exit(1);
        }
    }

    printf("%s Connected\n",rob_name);
    fflush(stdout);

    RobView robView(irSensorAngles, rob_name);


    // Connect event NewMessage to handler act()
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), &app, SLOT(act()));
    
    // create robot display widget

    // Connect event NewMessage to handler redrawRobot()
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), &robView, SLOT(redrawRobot()));
    
    robView.show();

    // process events
    return app.exec();
}

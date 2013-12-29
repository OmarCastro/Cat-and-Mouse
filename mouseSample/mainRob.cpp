
/* mainRob.C
 *
 * Basic Robot Agent
 * Very simple version for demonstration
 *
 * For more information about the CiberRato Robot Simulator 
 * please see http://microrato.ua.pt/ or contact us.
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "robfunc.h"




int main(int argc, char *argv[])
{
    char host[100]="localhost";
    char rob_name[20]="robsample";
    char typeStr[10]="cat";
    externalRobot robots[5];

    float lPow,rPow;
    int state=STOP, stoppedState=RUN, rob_id = 1;
    int beaconToFollow=0;
    int actionState = RUNNING;
    int type = 0;

    printf( " Sample Robot\n Copyright (C) 2001-2011 Universidade de Aveiro\n" );

     /* processing arguments */
    while (argc > 2) /* every option has a value, thus argc must be 1, 3, 5, ... */
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
        else if (strcmp(argv[1], "-type") == 0)
        {
           strncpy(typeStr, argv[2], 9);
           typeStr[9]='\0';
           type = (strcmp(typeStr,"mouse") == 0) ? 1:0;
        }
        else if (strcmp(argv[1], "-pos") == 0)
        {
            if(sscanf(argv[2], "%d", &rob_id)!=1)
               argc=0; /* error message will be printed */
        }
        else
        {
                break; /* the while */
        }
        argc -= 2;
        argv += 2;
    }

    if (argc != 1)
    {
        fprintf(stderr, "Bad number of parameters\n"
                "SYNOPSIS: mainRob [-host hostname] [-robname robotname] [-pos posnumber]\n");

        return 1;
    }

    /* Connect Robot to simulator */
    if(InitRobot(rob_name, rob_id, host,type)==-1)
    {
       printf( "%s Failed to connect\n", rob_name); 
       exit(1);
    }
    printf( "%s Connected as %s\n", rob_name, typeStr );
    state=STOP;
    while(1)
    {
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


        if(type == 0){  //if its a cat he is conditioned to tell its GPS position
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
                    if(pos.distance < 1){
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
    return 1;
}

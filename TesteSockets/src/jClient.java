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

import java.io.*;
import java.net.*;
import java.util.*;

import org.java_websocket.WebSocketImpl;

import ciberIF.*;

/**
 * example of a basic agent
 * implemented using the java interface library.
 */
public class jClient {

	public volatile static double lmotorJogador1 = 0;
	public volatile static double rmotorJogador1 = 0;
	public volatile static double lmotorJogador2 = 0;
	public volatile static double rmotorJogador2 = 0;
	
    ciberIF cif;
	public volatile static int rob_id = 0;
	public volatile static int rob_type = 0;
	public volatile static int HP = 500;

    
    enum State {RUN, WAIT, RETURN}

    public static void main(String[] args) {

    	String locale = System.getProperty("user.language")+"_"+
    	        System.getProperty("user.country");
        System.out.println(locale);
        

	String host, robName;
	int pos; 
	final int porta;
	int arg;

	//default values
	host = "localhost";
	robName = "jClient";
	pos = 1;
	


        // parse command-line arguments
	try {
	    arg = 0;
	    while (arg<args.length) {
		if(args[arg].equals("-pos")) {
			if(args.length > arg+1) {
				pos = Integer.valueOf(args[arg+1]).intValue();
				arg += 2;
			}
		}
		else if(args[arg].equals("-robname")) {
			if(args.length > arg+1) {
				robName = args[arg+1];
				arg += 2;
			}
		}
		else if(args[arg].equals("-host")) {
			if(args.length > arg+1) {
				host = args[arg+1];
				arg += 2;
			}
		}
		
		else if(args[arg].equals("-type")) {
			if(args.length > arg+1) {
				rob_type = args[arg+1].equals("mouse")?1:0;
				arg += 2;
			}
		}
		else throw new Exception();
	    }
	}
	catch (Exception e) {
		print_usage();
		return;
	}
	
	// create client
	jClient client = new jClient();

        client.robName = robName;

	// register robot in simulator
	client.cif.InitRobot(robName, pos, host,rob_type);
	
	
	//Inicar servidor
	porta = 8000;//-> Depois passar por linha de comandos
	Runnable r = new Runnable() 
	{
        public void run() 
        {
			try 
			{
				WebSocketImpl.DEBUG = true;
				Servidor s = new Servidor( porta );
				s.start();
				System.out.println( "Servidor tomou posse da porta: " + s.getPort() );
			} catch (UnknownHostException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				System.exit(-1);
			}
        }
    };
    
    //Iniciar thread onde correrá o servidor
    new Thread(r).start();
	
	// main loop
	client.mainLoop();
	
    }

    // Constructor
    jClient() {
	    cif = new ciberIF();
	    beacon = new beaconMeasure();

	    beaconToFollow = 0;
	    ground=-1;

            state = State.RUN;
    }

    /** 
     * reads a new message, decides what to do and sends action to simulator
     */
    public void mainLoop () {
	while(true) {
		cif.ReadSensors();
		decide();
	}
    }

    public void wander(boolean followBeacon) {
        
    	//Lê valores do servidor e toma a respectiva acção.
    	
    	
    	/*if(irSensor0>4.0 || irSensor1>4.0 ||  irSensor2>4.0) 
    	    cif.DriveMotors(0.1,-0.1);
        else if(irSensor1>1.0) cif.DriveMotors(0.1,0.0);
        else if(irSensor2>1.0) cif.DriveMotors(0.0,0.1);
        else if(followBeacon && beacon.beaconVisible && beacon.beaconDir > 20.0) 
	    cif.DriveMotors(0.0,0.1);
        else if(followBeacon && beacon.beaconVisible && beacon.beaconDir < -20.0) 
	    cif.DriveMotors(0.1,0.0);
        else cif.DriveMotors(0.1,0.1);*/
    	cif.DriveMotors(lmotorJogador2,rmotorJogador2);
    }

    /**
     * basic reactive decision algorithm, decides action based on current sensor values
     */
    public void decide() {
	    if(cif.IsObstacleReady(0))
		    irSensor0 = cif.GetObstacleSensor(0);
	    if(cif.IsObstacleReady(1))
		    irSensor1 = cif.GetObstacleSensor(1);
	    if(cif.IsObstacleReady(2))
		    irSensor2 = cif.GetObstacleSensor(2);

	    if(cif.IsCompassReady())
		    compass = cif.GetCompassSensor();
	    if(cif.IsGroundReady())
		    ground = cif.GetGroundSensor();

	    if(cif.IsBeaconReady(beaconToFollow))
		    beacon = cif.GetBeaconSensor(beaconToFollow);

	    x = cif.GetX();
	    y = cif.GetY();
	    dir = cif.GetDir();

            //System.out.println("Measures: ir0=" + irSensor0 + " ir1=" + irSensor1 + " ir2=" + irSensor2 + "\n");
            //System.out.println("Measures: x=" + x + " y=" + y + " dir=" + dir);

            //System.out.println(robName + " state " + state);

            switch(state) {
                 case RUN:    /* Go */
		     if( cif.GetVisitingLed() ) state = State.WAIT;
			 if(rob_type == 0){
				 if(cif.IsGPSReady()){
					 cif.Say(String.format( "p %f %f",cif.GetX(),cif.GetY()));
			      }
				 for(int i=0;i<5;i++){
		             int id = i+1;
		             if(rob_id == id) continue;
		             if(cif.NewMessageFrom(id)){
		                 String msg = cif.GetMessageFrom(id);
		                 if(msg.startsWith("m")){ //check if the message is about catching mouse
		                     int id_mouse= 0, id_cat = 0;
		                     Scanner scanner = new Scanner (msg);
		                     scanner.next (); // 2
		                     id_mouse = scanner.nextInt ();    // A 
		                     id_cat = scanner.nextInt ();    // A 
		                     scanner.close();
		                     beaconToFollow++;
		                     if(beaconToFollow >= cif.GetNumberOfBeacons()){
		                    	 cif.Finish();
		                     }
		                 }
		             }
		         }
			 } else {
				 for(int i=0;i<5;i++){
		             int id = i+1;
		             if(rob_id == id) continue;
		             if(cif.NewMessageFrom(id)){
		                 String msg = cif.GetMessageFrom(id);
		                 if(msg.startsWith("p")){ //check if the message is about position
		                	 double x= 0, y = 0;
		                     Scanner scanner = new Scanner (msg);
		                     scanner.next (); // 2
		                     x = scanner.nextDouble();    // A 
		                     y = scanner.nextDouble();
		                     scanner.close();// A sscanf(msg+2, "%lf %lf",&robots[i].x,&robots[i].y);
		                     if(cif.IsGPSReady()){
		                    	 double ix = cif.GetX();
		                    	 double iy = cif.GetY();
		                    	 double dist = (x - ix)*(x - ix)+(y - iy)*(y - iy);
		                    	 
		                    	 if(dist < 1.5){
		                             HP -= 100;
		                             if(HP < 0){
		                            	 cif.Say(String.format( "m %d %d",rob_id,id));
		                            	 cif.Finish();
		                             }
		                    	 }
		                     }
		                 }
		             }
		         }
			 }
		     
                         wander(true);
                     break;
		 case WAIT: /* Wait for others to visit target */
		     if(cif.GetReturningLed()) state = State.RETURN;

                     cif.DriveMotors(0.0,0.0);
                     break;
		 case RETURN: /* Return to home area */

		     if( cif.GetFinished() ) System.exit(0); /* Terminate agent */
		     if( ground == 1) { /* Finish */
                         cif.Finish();
                         System.out.println(robName + " found home at " + cif.GetTime() + "\n");
                     }
                     else {
                         wander(false);
                     }
                     break;

            }


            for(int i=1; i<6; i++)
              if(cif.NewMessageFrom(i))
                  System.out.println("Message: From " + i + " to " + robName + " : \"" + cif.GetMessageFrom(i)+ "\"");


	    if(cif.GetTime() % 2 == 0) {
	         cif.RequestIRSensor(0);
                 if(cif.GetTime() % 8 == 0 || state == State.RETURN )
                     cif.RequestGroundSensor();
                 else
                     cif.RequestBeaconSensor(beaconToFollow);
            }
            else {
            	String[] sensors = {"IRSensor1", "IRSensor2", "GPS"};
               cif.RequestSensors(sensors);
            }
    }

    static void print_usage() {
             System.out.println("Usage: java jClient [-robname <robname>] [-pos <pos>] [-host <hostname>[:<port>]]");
    }

    private String robName;
    private double irSensor0, irSensor1, irSensor2, compass;
    private beaconMeasure beacon;
    private int    ground;
    private boolean collision;
    private double x,y,dir;

    private State state;

    private int beaconToFollow;
};


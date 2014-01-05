# Cat and Mouse


### Authors

Name:           Omar Alejandro Castillo de Castro
Student Number: 080509158  (ei08158)


Name:           Vitor Hugo Goncalves dos Santos
Student Number: 090509059  (ei09059)
 


### Information

  Cat and Mouse is a robotic project on the Robotics course of the Integrated Master
  in Informatics and Computer Engineering at FEUP. It bases on the 
  CiberRato Robot Simulation Environment, in which simulates the movement
  of robots inside a labyrinth. There are 2 types of robots, the cats and the mice,
  the cats objective is to catch the mouse and the mouse is to avoid getting caught
  in the labyrinth.



### Operating System and Compiler

  To compile the code it is required to have the development version of Qt libraries
  release 4.x, it is not recommended to compile with Qt 5.x as of 30/12/2013 it is not 
  fully backward compactible with the Qt 4 library
  It is also required to have qmake installed.
  
  Any OS that have the development version of Qt libraries is enough, 
  though it was only tested on Ubuntu 13.10 and 12.04, so the usage of a linux 
  distribution is recommended.


### Contents

  simulator/           The simulator source code
  Viewer/              The Visualizer source code
  logplayer/           The logplayer source code
  AI_code/             Graphical robot agent (C++) source code
  Labs/                examples of labyrinths used in previous competitions

  README               This README file
  README.md            Github README file

  startAll             Startup script that runs the simulator, the 
                         visualizer and 5 GUISamples
  startSimViewer       Startup script that runs the simulator and the Viewer


### Compilation


First run:

    qmake --version

and check if it uses Qt 4 or Qt 5, most recent linux versions use Qt 5 if its Qt 4
modify the line in the Makefile

    QMAKE = qmake-qt4

to

    QMAKE = qmake

after cheching the Qt version in qmake, run:

    make

if it goes well, run:

    ./startAll

to run the simulator filled with robots; if the simulator is enough, run

    ./startSimViewer



### Execution of the application


    ./startAll

runs the simulator filled with robots

    ./startSimViewer

runs the simulator without robots, it needs to be added manually.
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

/* sampapp.h
 *
 * Basic Robot Agent
 * simple version for demonstration
 *
 * For more information about the CiberRato Robot Simulator 
 * please see http://microrato.ua.pt/ or contact us.
 */

#include <qapplication.h>

/* SampApp class declaration */
class SampApp : public QApplication
{
    Q_OBJECT
public:
    SampApp(int &argc, char*argv[], char *robot_name, int robot_type);
    ~SampApp();
public slots:
    void act(void);
private:
    char rob_name[20];
    int rob_type;

    int beaconToFollow; //used to support a simple sequential visiting strategy
};


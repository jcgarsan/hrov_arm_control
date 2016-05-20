/* 
 * Copyright (c) 2016 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Author:
 *     Juan Carlos García
 */ 

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include "../include/hrov_arm_control/hrov_arm_control.h"

/*#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <cmath>  */


using namespace std;



Hrov_arm_control::Hrov_arm_control()
{
	//initializing values

}

Hrov_arm_control::~Hrov_arm_control()
{
	//Destructor
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hrov_arm_control");
  Hrov_arm_control hrov_arm_control;
  ros::spin();
}



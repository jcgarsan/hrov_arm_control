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

#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <cmath>

#define num_sensors			 2		// 0 = is there an alarm?, 1 = surface, 2 = seafloor

//DEBUG Flags
#define DEBUG_FLAG_SAFETY	 0
#define DEBUG_FLAG_USER		 0
#define DEBUG_FLAG_ARM_INPUT 1

using namespace std;


Hrov_arm_control::Hrov_arm_control()
{
	//initializing values
	for (int i=0; i<3; i++)
		armInput[i] = 0.0;

	for (int i=0; i<=num_sensors; i++)
		safetyMeasureAlarm.data.push_back(0);
	for (int i=0; i<2; i++)
		userControlAlarm.data.push_back(0);

	//Subscriber initialization (sensors)
	sub_safetyMeasures = nh.subscribe<std_msgs::Int8MultiArray>("safetyMeasuresAlarm", 1, &Hrov_arm_control::safetyMeasuresCallback, this);
	sub_userControl = nh.subscribe<std_msgs::Int8MultiArray>("userControlAlarm", 1, &Hrov_arm_control::userControlCallback, this);
	sub_armInput = nh.subscribe<std_msgs::Float64MultiArray>("g500/arm_input", 1, &Hrov_arm_control::armInputCallback, this);

	//Arm control
	robot = new ARM5Arm(nh, "uwsim/joint_state", "uwsim/joint_state_command");
}


Hrov_arm_control::~Hrov_arm_control()
{
	//Destructor
}


void Hrov_arm_control::safetyMeasuresCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
	for (int i=0; i<=num_sensors; i++)
		safetyMeasureAlarm.data[i] = msg->data[i];

	if (DEBUG_FLAG_SAFETY)
	{
		cout << "safetyMeasureAlarm: [";
		for (int i=0; i<=num_sensors; i++)
			cout << (int) safetyMeasureAlarm.data[i] << ",";
		cout << "]" << endl;
	}
}


void Hrov_arm_control::userControlCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
	for (int i=0; i<2; i++)
		userControlAlarm.data[i] = msg->data[i];

	if (userControlAlarm.data[1])
		armControl();

	if (DEBUG_FLAG_USER)
		cout << "userControlCallback: [" << (int) userControlAlarm.data[0] << ", " << (int) userControlAlarm.data[1] << "]" << endl;
}


void Hrov_arm_control::armInputCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for (int i=0; i<3; i++)
		armInput[i] = msg->data[i];

	if (DEBUG_FLAG_ARM_INPUT)
	{
		cout << "armInput: [";
		for (int i=0; i<3; i++)
			cout << armInput[i] << "; ";
		cout << "]" << endl;
	}
}


void Hrov_arm_control::armControl()
{
	vpColVector desiredVel(6);

	desiredVel[0] = armInput[0];
	desiredVel[1] = armInput[1];
	desiredVel[2] = armInput[2];

	desiredVel[3] = 0;
	desiredVel[4] = 0;
	desiredVel[5] = 0;

	robot->setCartesianVelocity(desiredVel);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hrov_arm_control");
  Hrov_arm_control hrov_arm_control;
  ros::spin();
}



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

#define num_sensors			2		// 0 = is there an alarm?, 1 = surface, 2 = seafloor

//DEBUG Flags
#define DEBUG_FLAG_SAFETY	0
#define DEBUG_FLAG_USER		0


using namespace std;


Hrov_arm_control::Hrov_arm_control()
{
	//initializing values
	robotControl = true;

	for (int i=0; i<=num_sensors; i++)
		safetyMeasureAlarm.data.push_back(0);
	for (int i=0; i<2; i++)
		userControlAlarm.data.push_back(0);

	//Subscriber initialization (sensors)
	sub_safetyMeasures = nh.subscribe<std_msgs::Int8MultiArray>("safetyMeasuresAlarm", 1, &Hrov_arm_control::safetyMeasuresCallback, this);
	sub_userControl = nh.subscribe<std_msgs::Int8MultiArray>("userControlAlarm", 1, &Hrov_arm_control::userControlCallback, this);

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

	if (DEBUG_FLAG_USER)
		cout << "userControlCallback: [" << (int) userControlAlarm.data[0] << ", " << (int) userControlAlarm.data[1] << "]" << endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hrov_arm_control");
  Hrov_arm_control hrov_arm_control;
  ros::spin();
}



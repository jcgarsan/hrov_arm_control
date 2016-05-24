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
#define DEBUG_FLAG_ARM_DATA  1


using namespace std;


Hrov_arm_control::Hrov_arm_control()
{
	//initializing values
	robotControl = true;
	armMoving	 = false;	

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
		armInput[i] = msg->data[i]/10;

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
	vpColVector current_joints(5), send_joints(5);

	if(robot->getJointValues(current_joints))
	{		
		bMe=robot->directKinematics(current_joints);
		if (DEBUG_FLAG_ARM_DATA)
			cout << "after bMe" << endl << bMe << endl;
	}
	
	//Calculate the base-end effector matrix
	if (!armMoving)
	{
		desired_bMe = bMe;
		desired_bMe[0][3]-= armInput[0]; //currentPosition.pose.position.x;
		desired_bMe[1][3]-= armInput[2]; //currentPosition.pose.orientation.z; 
		desired_bMe[2][3]-= armInput[1]; //currentPosition.pose.position.y;
		next_joints = robot->armIK(desired_bMe);
		if (DEBUG_FLAG_ARM_DATA)
		{
			cout << "Desired joints" << endl << next_joints << endl;
			cout << "Desired bMe" << endl << desired_bMe << endl;
		}
	}			
		
	//If valid joints and reasonable new position ... ask to MOVE
	if ((next_joints[0] > -1.57) and (next_joints[0] < 2.1195) and (next_joints[1] > 0) and \
		(next_joints[1] < 1.58665) and (next_joints[2] > 0) and (next_joints[2] < 2.15294))			// join limits
	{ //dist (m) entre current y desire
		if (((std::abs(desired_bMe[0][3] - bMe[0][3]) < 1.5) and \
			(std::abs(desired_bMe[1][3] - bMe[1][3]) < 1.5) and \
			(std::abs(desired_bMe[2][3] - bMe[2][3]) < 1.5)) and
			((std::abs(desired_bMe[0][3] - bMe[0][3]) > 0) or \
			(std::abs(desired_bMe[1][3] - bMe[1][3]) > 0) or \
			(std::abs(desired_bMe[2][3] - bMe[2][3]) > 0)))
		{
			armMoving = true;
			ROS_INFO("Moving...");
		}
		else
			ROS_INFO("Error: New position too far form the original position.");
	}
	else
		ROS_INFO("Error: Unreachable position.");			
	
	//Send the parameters
	if(armMoving)
	{
		//Check if it's almost there
		if((std::abs(desired_bMe[0][3] - bMe[0][3]) > 0.01) || \
			(std::abs(desired_bMe[1][3] - bMe[1][3]) > 0.01) || \
			(std::abs(desired_bMe[2][3] - bMe[2][3]) > 0.01))
		{
//			ROS_INFO("Info: Moving to desired position.");
			send_joints[0]=next_joints[0]-current_joints[0];
			send_joints[1]=next_joints[1]-current_joints[1];
			send_joints[2]=next_joints[2]-current_joints[2];
			ROS_INFO_STREAM (send_joints[0]<<"::"<<send_joints[1]<<"::"<<send_joints[2]);
		}
		else
		{
//			ROS_INFO("Info: Position reached");
			send_joints[0]=0;
			send_joints[1]=0;
			send_joints[2]=0;
			armMoving=false;
		}
	}
	else
	{
//		ROS_INFO("Info: arm is not moving.");
		send_joints[0]=0;
		send_joints[1]=0;
		send_joints[2]=0;		
	}
	
	//Gripper rotation & apperture control
/*	if (gripperRotation == 0)
		send_joints[3] = 0;
	else if (gripperRotation == 1)
		send_joints[3] = 0.05;
	else
		send_joints[3] = -0.05;
	if (gripperApperture == 0)
		send_joints[4] = 0;
	else if (gripperApperture == 1)
		send_joints[4] = 0.05;
	else
		send_joints[4] = -0.05;		*/

	robot->setJointVelocity(send_joints);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hrov_arm_control");
  Hrov_arm_control hrov_arm_control;
  ros::spin();
}



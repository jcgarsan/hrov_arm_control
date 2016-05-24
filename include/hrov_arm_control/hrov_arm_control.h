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

//ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <mar_robot_arm5e/ARM5Arm.h>


using namespace std;


class Hrov_arm_control
{

	public:
		Hrov_arm_control();
		~Hrov_arm_control();
		
		bool						robotControl;
		bool 						armMoving;

		double						armInput[3];
		
		std_msgs::Int8MultiArray	safetyMeasureAlarm;
		std_msgs::Int8MultiArray	userControlAlarm;

		ARM5Arm 					*robot;
		vpHomogeneousMatrix 		desired_bMe, bMe;
		vpColVector 				next_joints;


	private:
		void safetyMeasuresCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);
		void userControlCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);
		void armInputCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
		void armControl();

		ros::NodeHandle				nh;

		ros::Subscriber				sub_safetyMeasures;
		ros::Subscriber				sub_userControl;
		ros::Subscriber				sub_armInput;

		tf::Transform 				transform_init, transform_new;
		tf::StampedTransform 		transform;
		tf::Quaternion				q_init, q_new;
		tf::TransformBroadcaster	br;
		tf::TransformListener		*listener;
};

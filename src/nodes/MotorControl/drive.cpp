/*
10/10/2019
author : Charles Goudaert	contact : charles.goudaert@isen.yncrea.fr
Based on the RoboteQ linux API
*/
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <cmath>
#include <complex>

//Relative to the RoboteQ linux API
#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;


//Create 2 driver objects for both front motors and back ones
RoboteqDevice frontDriver;
RoboteqDevice backDriver;

#define WHEEL_RADIUS 0.05
#define WTOW_LENGHT 0.20
#define WTO_WIDTH 0.22


/*
setCommands is the callback fonction called when a Twist message is published on the command velocity topic (cmd_vel).
Input: 2 3 dimensions vectors -> linear and angular 
It uses x,y from linear and z from angular to calculate each motor's speed to get the robot move as desired. 
*/
void setCommands(const geometry_msgs::Twist& msg)
{
	float frontRightSpeed;
	float frontLeftSpeed;
	float backRightSpeed;
	float backLeftSpeed;

	float thetad = msg.angular.z;

	//Equations for mecanum wheeled robot

	frontLeftSpeed = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - thetad * (WTOW_LENGHT + WTO_WIDTH));
	frontRightSpeed = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y + thetad * (WTOW_LENGHT + WTO_WIDTH));
	backLeftSpeed = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y - thetad * (WTOW_LENGHT + WTO_WIDTH));
	backRightSpeed = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y + thetad * (WTOW_LENGHT + WTO_WIDTH));


	cout << "FL motor speed :" << frontLeftSpeed << " FR motor speed :" << frontRightSpeed << endl;
	cout << "BL motor speed :" << backLeftSpeed << " BR motor speed :" << backRightSpeed << endl;

	if(msg.linear.z != 0)
	{
		if(msg.linear.z == 2)
		{
			frontDriver.SetCommand(_GO, 1, 1000);
			frontDriver.SetCommand(_GO, 2, -1000);
			backDriver.SetCommand(_GO, 1, -1000);
			backDriver.SetCommand(_GO, 2, 1000);
			cout << "front right" << endl;
		}
		if(msg.linear.z == 1)
		{
			frontDriver.SetCommand(_GO, 1, -1000);
			frontDriver.SetCommand(_GO, 2, 1000);
			backDriver.SetCommand(_GO, 1, 1000);
			backDriver.SetCommand(_GO, 2, -1000);
			cout << "front left" << endl;
		}
		if(msg.linear.z == 3)
		{
			frontDriver.SetCommand(_GO, 1, 1000);
			frontDriver.SetCommand(_GO, 2, 1000);
			backDriver.SetCommand(_GO, 1, 1000);
			backDriver.SetCommand(_GO, 2, 1000);
			cout << "back right" << endl;
		}
		if(msg.linear.z == 4)
		{
			frontDriver.SetCommand(_GO, 1, -1000);
			frontDriver.SetCommand(_GO, 2, -1000);
			backDriver.SetCommand(_GO, 1, -1000);
			backDriver.SetCommand(_GO, 2, -1000);
			cout << "back left" << endl;
		}
	}

	else
	{

		//Send computed speeds to the front driver
		frontDriver.SetCommand(_GO, 1, frontRightSpeed * 1000/8.8);
		frontDriver.SetCommand(_GO, 2, frontLeftSpeed * 1000/8.8);
	
		//Send computed speeds to the back driver
		backDriver.SetCommand(_GO, 1, backRightSpeed * 1000/8.8);
		backDriver.SetCommand(_GO, 2, backLeftSpeed * 1000/8.8);

	}
	
}


int main(int argc, char *argv[])
{
	//Init the ROS node named "drive"
	ros::init(argc, argv, "drive");
	ros::NodeHandle n;
	//Subscribe to cmd_vel topic and call the setCommands function 
	ros::Subscriber sub = n.subscribe("cmd_vel", 100, setCommands);


	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;


	//Default ports for the drivers
	string frontDriver_port = "/dev/ttyACM1";
	string backDriver_port = "/dev/ttyACM0";
	//Ports for the drivers reasigned if needed in the launch file
	n.getParam("frontDriver_port", frontDriver_port);
	n.getParam("backDriver_port", backDriver_port);

	cout << endl << "RoboteQ Motor Drivers setup :" << endl;
	cout << "------------------------" << endl;

	int status1 = frontDriver.Connect(frontDriver_port);
	int status2 = backDriver.Connect(backDriver_port);

	//If we have an issue with one of the drivers
	if (status1 != RQ_SUCCESS && status2 != RQ_SUCCESS)
	{
		//Exit the program
		cout << "Error connecting to devices: " << status1 + status2 << "/2." << endl;
		return 1;
	}


	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	double vx;
	double vy;
	double vth;

	int rpm_frontLeft;
	int rpm_frontRight;
	int rpm_backLeft;
	int rpm_backRight;

	ros::Time current_time, last_time;
  	current_time = ros::Time::now();
  	last_time = ros::Time::now();

	ros::Rate r(1.0);

	while (ros::ok())
	{
		ros::spinOnce();

		//Get values of encoders in rpm
		frontDriver.GetValue(_ABSPEED, 2, rpm_frontLeft);
		frontDriver.GetValue(_ABSPEED, 1, rpm_frontRight);
		backDriver.GetValue(_ABSPEED, 2, rpm_backLeft);
		backDriver.GetValue(_ABSPEED, 1, rpm_backRight);

		//convert rpm into speed TODO

		vx = (rpm_frontLeft + rpm_frontRight + rpm_backLeft + rpm_backRight)/4;
		vy = (- rpm_frontLeft + rpm_frontRight + rpm_backLeft - rpm_backRight)/4;
		vth = (rpm_backRight - rpm_frontLeft) / (2*(WTOW_LENGHT + WTO_WIDTH));


		current_time = ros::Time::now();

		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//publish the message
		odom_pub.publish(odom);

		last_time = current_time;
		r.sleep();
	}
	

	
	cout << "Disconnecting Drivers ..." << endl;
	frontDriver.Disconnect();
	backDriver.Disconnect();
return 0;
}

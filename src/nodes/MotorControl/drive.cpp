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

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

using namespace std;


//Create 2 driver objects for both front motors and back ones
RoboteqDevice frontDriver;
RoboteqDevice backDriver;


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

	float Vd = sqrt(pow(msg.linear.x, 2) + pow(msg.linear.y, 2));
	float thetad = msg.angular.z;


	float wheelRadius = 0.05;
	float wTowLenght = 0.20;
	float wTowWidth = 0.22;

	//Equations for mecanum wheeled robot

	frontLeftSpeed = (1 / wheelRadius) * (msg.linear.x - msg.linear.y - thetad * (wTowLenght + wTowWidth));
	frontRightSpeed = (1 / wheelRadius) * (msg.linear.x + msg.linear.y + thetad * (wTowLenght + wTowWidth));
	backLeftSpeed = (1 / wheelRadius) * (msg.linear.x + msg.linear.y - thetad * (wTowLenght + wTowWidth));
	backRightSpeed = (1 / wheelRadius) * (msg.linear.x - msg.linear.y + thetad * (wTowLenght + wTowWidth));
	

	cout << "Vd: " << Vd << " thetad: " << thetad << endl;


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
		frontDriver.SetCommand(_GO, 2, frontLeftSpeed * 1000)/8.8;
	
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



	ros::spin();

	frontDriver.Disconnect();
	backDriver.Disconnect();
	return 0;
}

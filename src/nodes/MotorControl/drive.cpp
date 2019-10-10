/*
10/10/2019
author : Charles Goudaert	contact : charles.goudaert@isen.yncrea.fr
Based on the RoboteQ linux API
*/
#include <iostream>
#include <stdio.h>
#include <string.h>

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
	int frontRightSpeed;
	int frontLeftSpeed;
	int backRightSpeed;
	int backLeftSpeed;

	//Equations for mecanum wheeled robot
	frontRightSpeed = msg.linear.x - msg.linear.y + msg.angular.z;
	frontLeftSpeed = msg.linear.x + msg.linear.y + msg.angular.z;
	backRightSpeed = msg.linear.x + msg.linear.y - msg.angular.z;
	backLeftSpeed = msg.linear.x - msg.linear.y - msg.angular.z;

	cout << "FL motor speed :" << frontLeftSpeed << " FR motor speed :" << frontRightSpeed << endl;

	//Send computed speeds to the front driver
	frontDriver.SetCommand(_GO, 1, frontRightSpeed);
	frontDriver.SetCommand(_GO, 2, frontRightSpeed);
	
	cout << endl << "BL motor speed :" << backLeftSpeed << " BR motor speed :" << backRightSpeed << endl;

	//Send computed speeds to the back driver
	backDriver.SetCommand(_GO, 1, backRightSpeed);
	backDriver.SetCommand(_GO, 2, backRightSpeed);
	
}



int main(int argc, char *argv[])
{
	//Init the ROS node named "drive"
	ros::init(argc, argv, "drive");
	ros::NodeHandle n;
	//Subscribe to cmd_vel topic and call the setCommands function 
	ros::Subscriber sub = n.subscribe("cmd_vel", 10, setCommands);

	//Default ports for the drivers
	string frontDriver_port = "/dev/ttyACM0";
	string backDriver_port = "/dev/ttyACM1";
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
		cout << "Error connecting to devices: " << status1 << "/" << status2 << "." << endl;
		return 1;
	}



	ros::spin();

	frontDriver.Disconnect();
	backDriver.Disconnect();
	return 0;
}

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
#include <algorithm>

//Relative to the RoboteQ linux API
#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

#include "RobotSpecs.h"
#include "heron/Encoders.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;

class Driver
{
	private:
		ros::NodeHandle n;

		ros::Subscriber sub;
		ros::Publisher encoders_pub;

		//Create 2 driver objects for both front motors and back ones
		RoboteqDevice frontDriver;
		RoboteqDevice backDriver;

		// custom msg
		heron::Encoders diff_encs;

		double saturate(double value, double sat)
		{
			if(value > sat)
			{
				return sat;
			}
			else
			{
				return value;
			}
			
		}

		struct WheelsEncoders
		{
			int Fl;
			int Fr;
			int Bl;
			int Br;

			int tmp_Fl;
			int tmp_Fr;
			int tmp_Bl;
			int tmp_Br;
		};

		// tmp variable to stoge encoders data
		WheelsEncoders wencs;


	public:
		Driver()
		{	
			cout << endl << "Initialize Drivers" << endl;
			//Subscribe to cmd_vel topic and call the setCommands function 
			sub = n.subscribe("cmd_vel", 1, &Driver::setCommands, this);
			encoders_pub = n.advertise<heron::Encoders>("sensor_encs", 10);

			wencs.tmp_Fl = 0;
			wencs.tmp_Fr = 0;
			wencs.tmp_Bl = 0;
			wencs.tmp_Br = 0;
		}
		~Driver() {}

		/*
		Initialize connection between RoboteQ drivers and computer.
		*/
		int connect()
		{
			//Default ports for the drivers
			string frontDriver_port = "/dev/roboteq_front";
			string backDriver_port = "/dev/roboteq_back";

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
			else
			{
				return 0;
			}	
		}

		/*
		Disconnect RoboteQ drivers from computer.
		*/
		void disconnect()
		{
			cout << "Disconnecting Drivers ..." << endl;
			frontDriver.Disconnect();
			backDriver.Disconnect();
		}


		void pubEncoders()
		{

			const int max = (MOTOR_OUTPUT_SHAFT_MAX_RPM / 60) * ENCODERS_COUNTABLE_EVENTS_OUTPUT_SHAFT / ODOM_RATE;

			//Get values of encoders and calculate difference since the last time the function was called		
			frontDriver.GetValue(_ABCNTR, 2, wencs.Fl);
			frontDriver.GetValue(_ABCNTR, 1, wencs.Fr);
			backDriver.GetValue(_ABCNTR, 2, wencs.Bl);
			backDriver.GetValue(_ABCNTR, 1, wencs.Br);

			diff_encs.EncFl = wencs.tmp_Fl - wencs.Fl;
			diff_encs.EncFr = wencs.tmp_Fr - wencs.Fr;
			diff_encs.EncBl = wencs.tmp_Bl - wencs.Bl;
			diff_encs.EncBr = wencs.tmp_Br - wencs.Br;

			if(diff_encs.EncFl < max
			&& diff_encs.EncFr < max
			&& diff_encs.EncBl < max
			&& diff_encs.EncBr < max)
			{
				wencs.tmp_Fl = wencs.Fl;
				wencs.tmp_Fr = wencs.Fr;
				wencs.tmp_Bl = wencs.Bl;
				wencs.tmp_Br = wencs.Br;
			}
			else
			{
				ROS_INFO("Encoders Jump detected");
			}
			
			encoders_pub.publish(diff_encs);
		}

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

			frontLeftSpeed = saturate((1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y + thetad * (WTOW_LENGHT + WTO_WIDTH)) * 1000/8.8 , 1000); 
			frontRightSpeed = saturate((1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - thetad * (WTOW_LENGHT + WTO_WIDTH)) * 1000/8.8 , 1000);
			backLeftSpeed = saturate((1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y - thetad * (WTOW_LENGHT + WTO_WIDTH)) * 1000/8.8 , 1000);
			backRightSpeed = saturate((1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y + thetad * (WTOW_LENGHT + WTO_WIDTH)) * 1000/8.8 , 1000);

			

			// usefull for debuging purpose
			// cout << "FL motor speed :" << frontLeftSpeed << " FR motor speed :" << frontRightSpeed << endl;
			// cout << "BL motor speed :" << backLeftSpeed << " BR motor speed :" << backRightSpeed << endl;


			//Send computed speeds to the front driver
			frontDriver.SetCommand(_GO, 1, frontRightSpeed);
			frontDriver.SetCommand(_GO, 2, frontLeftSpeed);

			//Send computed speeds to the back driver
			backDriver.SetCommand(_GO, 1, backRightSpeed);
			backDriver.SetCommand(_GO, 2, backLeftSpeed);

		}

};//End of class Drivers




int main(int argc, char *argv[])
{
	//Init the ROS node named "drive"
	ros::init(argc, argv, "drive");
	
	Driver drivers;

	drivers.connect();


	ros::Rate r(100);
	while (ros::ok())
	{
		ros::spinOnce();

		drivers.pubEncoders();
		
		r.sleep();
	}
	// ros::spin();

	drivers.disconnect();

	return 0;
}



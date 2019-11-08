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

#include "RobotSpecs.h"
#include "heron/Encoders.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;

class Driver
{
	private:
		ros::NodeHandle n;

		//Subscribe to cmd_vel topic and call the setCommands function 
		ros::Subscriber sub;
		ros::Publisher encoders_pub;

		//Create 2 driver objects for both front motors and back ones
		RoboteqDevice frontDriver;
		RoboteqDevice backDriver;

		// custom msg
		heron::Encoders encs;

		int tmp_encFl = 0;
		int tmp_encFr = 0;
		int tmp_encBl = 0;
		int tmp_encBr = 0;



	public:
		Driver()
		{	
			cout << "Initialize Drivers" << endl;
			sub = n.subscribe("cmd_vel", 100, &Driver::setCommands, this);
			encoders_pub = n.advertise<heron::Encoders>("sensor_encs", 10);
		}
		~Driver() {}

		int connect()
		{
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
			else
			{
				if (frontDriver.IsConnected() && backDriver.IsConnected())
				{
					cout << "RoboteQ drivers working fine" << endl;
					return 0;
				}
				else
				{
					return 1;
				}
				
			}
			
		}

		void disconnect()
		{
			cout << "Disconnecting Drivers ..." << endl;
			frontDriver.Disconnect();
			backDriver.Disconnect();
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

			int enc_value;

			float thetad = msg.angular.z;

			//Equations for mecanum wheeled robot

			frontLeftSpeed = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - thetad * (WTOW_LENGHT + WTO_WIDTH));
			frontRightSpeed = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y + thetad * (WTOW_LENGHT + WTO_WIDTH));
			backLeftSpeed = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y - thetad * (WTOW_LENGHT + WTO_WIDTH));
			backRightSpeed = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y + thetad * (WTOW_LENGHT + WTO_WIDTH));


			cout << "FL motor speed :" << frontLeftSpeed << " FR motor speed :" << frontRightSpeed << endl;
			cout << "BL motor speed :" << backLeftSpeed << " BR motor speed :" << backRightSpeed << endl;


			//Send computed speeds to the front driver
			frontDriver.SetCommand(_GO, 1, frontRightSpeed * 1000/8.8);
			frontDriver.SetCommand(_GO, 2, frontLeftSpeed * 1000/8.8);

			//Send computed speeds to the back driver
			backDriver.SetCommand(_GO, 1, backRightSpeed * 1000/8.8);
			backDriver.SetCommand(_GO, 2, backLeftSpeed * 1000/8.8);

			//Get values of encoders in rpm (MAX 84rpm)
			
			frontDriver.GetValue(_ABCNTR, 2, enc_value);
			encs.EncFl = tmp_encFl - enc_value;
			tmp_encFl = enc_value;

			frontDriver.GetValue(_ABCNTR, 1, enc_value);
			encs.EncFr = tmp_encFr - enc_value;
			tmp_encFr = enc_value;

			backDriver.GetValue(_ABCNTR, 2, enc_value);
			encs.EncBl = tmp_encBl - enc_value;
			tmp_encBl = enc_value;

			backDriver.GetValue(_ABCNTR, 1, enc_value);
			encs.EncBr = tmp_encBr - enc_value;
			tmp_encBr = enc_value;

			encoders_pub.publish(encs);
		}

};//End of class Drivers




int main(int argc, char *argv[])
{
	//Init the ROS node named "drive"
	ros::init(argc, argv, "drive");
	
	Driver drivers;

	drivers.connect();


	// ros::Rate r(200);
	// while (ros::ok())
	// {
	// 	ros::spinOnce();
	// 	r.sleep();
	// }
	ros::spin();

	drivers.disconnect();

	return 0;
}



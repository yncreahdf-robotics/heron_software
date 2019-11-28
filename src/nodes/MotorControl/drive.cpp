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
		heron::Encoders encs_msg;

		struct WheelsEncoders
		{
			int Fl, Fr, Bl, Br;
		};

		// tmp variable to stoge encoders data
		WheelsEncoders encs;
		WheelsEncoders tmp_encs;


		/* 
		Saturate motor command 
		*/
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
			
		}// End saturate


		/* 
		Initialize the encoders by loading the current 
		value to tmp variables
		*/
		void initEncoders()
		{
			frontDriver.GetValue(_ABCNTR, 2, tmp_encs.Fl);
			frontDriver.GetValue(_ABCNTR, 1, tmp_encs.Fr);
			backDriver.GetValue(_ABCNTR, 2, tmp_encs.Bl);
			backDriver.GetValue(_ABCNTR, 1, tmp_encs.Br);
		}// End initEncoders


	public:
		Driver()
		{	
			cout << endl << "Initialize Drivers" << endl;
			//Subscribe to cmd_vel topic and call the setCommands function 
			sub = n.subscribe("cmd_vel", 1, &Driver::setCommands, this);
			encoders_pub = n.advertise<heron::Encoders>("sensor_encs", 10);
		}// End Constructor

		~Driver() {}// End Destructor

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
				initEncoders();
				return 0;
			}	
		}// End connect


		/*
		Disconnect RoboteQ drivers from computer.
		*/
		void disconnect()
		{
			cout << "Disconnecting Drivers ..." << endl;
			frontDriver.Disconnect();
			backDriver.Disconnect();
		}// End disconnect


		/* 
		Get the absolute encoder value from the driver 
		Filter the values to get rid of the "jumps"
		Publish the diff over a topic to the odom 
		*/
		void pubEncoders()
		{
			WheelsEncoders diff;

			//Get values of encoders through the drivers		
			frontDriver.GetValue(_ABCNTR, 2, encs.Fl);
			frontDriver.GetValue(_ABCNTR, 1, encs.Fr);
			backDriver.GetValue(_ABCNTR, 2, encs.Bl);
			backDriver.GetValue(_ABCNTR, 1, encs.Br);

			// Compute the difference since the last time the function was called
			diff.Fl = tmp_encs.Fl - encs.Fl;
			diff.Fr = tmp_encs.Fr - encs.Fr;
			diff.Bl = tmp_encs.Bl - encs.Bl;
			diff.Br = tmp_encs.Br - encs.Br;
			ROS_INFO("Encoders Fl%d Fr%d Bl%d Br%d", diff.Fl, diff.Fr, diff.Bl, diff.Br);

			// if datas are plosible (no jump)
			if(diff.Fl < MAX_DELTA_ENCODERS
			&& diff.Fr < MAX_DELTA_ENCODERS
			&& diff.Bl < MAX_DELTA_ENCODERS
			&& diff.Br < MAX_DELTA_ENCODERS)
			{
				// Update the values to send to odom
				encs_msg.EncFl = diff.Fl;
				encs_msg.EncFr = diff.Fr;
				encs_msg.EncBl = diff.Bl;
				encs_msg.EncBr = diff.Br;
				
				// Store the current value for next loop
				tmp_encs.Fl = encs.Fl;
				tmp_encs.Fr = encs.Fr;
				tmp_encs.Bl = encs.Bl;
				tmp_encs.Br = encs.Br;
			}
			else
			{
				// Don't change the values to send to odom
				// So it sends the previous one 

				ROS_INFO("Encoders Jump detected");

				// catch up the value error on the failing encoder(s)
				if(encs.Fl > MAX_DELTA_ENCODERS)
				{
					tmp_encs.Fl = encs.Fl;
				}
				if(encs.Fr > MAX_DELTA_ENCODERS)
				{
					tmp_encs.Fr = encs.Fr;
				}
				if(encs.Bl > MAX_DELTA_ENCODERS)
				{
					tmp_encs.Bl = encs.Bl;
				}
				if(encs.Br > MAX_DELTA_ENCODERS)
				{
					tmp_encs.Br = encs.Br;
				}

			}
			cout << "Published Encoders: " << encs_msg.EncFl << ";" << encs_msg.EncFr << ";" << encs_msg.EncBl << ";" << encs_msg.EncBr << endl;
			// Publish the datas
			encoders_pub.publish(encs_msg);
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

	drivers.disconnect();

	return 0;
}



#include <iostream>
#include <stdio.h>
#include <string.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

using namespace std;

int MotorControllerCommand(string port, int channel) 
{
	cout << endl << "Motor Controller Command:" << endl;
	cout << "------------------------" << endl;
	string response = "";
	RoboteqDevice device;
	int status = device.Connect(port);

	if (status != RQ_SUCCESS)
	{
		cout << "Error connecting to device: " << status << "." << endl;
		return 1;
	}

	int speed;
	speed =1;
	
	while(speed != 0){
		cin >> speed;
		device.SetCommand(_GO, channel, speed);
	}

	device.Disconnect();
	return 0;
}


int MotorControllerSample(string port) 
{
	cout << endl << "Motor Controller Sample:" << endl;
	cout << "------------------------" << endl;
	string response = "";
	RoboteqDevice device;
	int status = device.Connect(port);

	if (status != RQ_SUCCESS)
	{
		cout << "Error connecting to device: " << status << "." << endl;
		return 1;
	}


	int canNodeID;
	cout << "- Read CAN Node ID: GetConfig(_CNOD, 1)...";
	if ((status = device.GetConfig(_CNOD, 1, canNodeID)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "returned --> " << canNodeID << endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	cout << "- Set CAN Node ID: SetConfig(_CNOD, 1, "<< canNodeID<<")...";
	if ((status = device.SetConfig(_CNOD, 1, canNodeID)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	cout << "- Set User Variable 10: SetCommand(_VAR, 10, 100)...";
	if ((status = device.SetCommand(_VAR, 10, 100)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	int result;
	cout << "- Read User Variable 10: GetValue(_VAR, 10)...";
	if ((status = device.GetValue(_VAR, 10, result)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "returned --> " << result << endl;

	device.Disconnect();
	return 0;
}
int MagSensorSample(string port)
{
	cout << endl << "MagSensor Sample:" << endl;
	cout << "-----------------" << endl;
	string response = "";
	RoboteqDevice device;
	int status = device.Connect(port);

	if (status != RQ_SUCCESS)
	{
		cout << "Error connecting to device: " << status << "." << endl;
		return 1;
	}

	int canNodeID;
	cout << "- Read CAN Node ID: GetConfig(_CNOD, 1)...";
	if ((status = device.GetConfig(_CNOD, 1, canNodeID)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "returned --> " << canNodeID << endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	cout << "- Set CAN Node ID: SetConfig(_CNOD, 1, " << canNodeID << ")...";
	if ((status = device.SetConfig(_CNOD, 1, canNodeID)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	cout << "- Set User Variable 10: SetCommand(_VAR, 10, 100)...";
	if ((status = device.SetCommand(_VAR, 10, 100)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	int result;
	cout << "- Read User Variable 10: GetValue(_VAR, 10)...";
	if ((status = device.GetValue(_VAR, 10, result)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "returned --> " << result << endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	//Check track detection.
	cout << endl << "Monitor track detection (CTRL+C for Exit):" << endl;
	cout << "------------------------------------------" << endl;
	int lastTrackResult = -1;
	while (true)
	{
		if (device.GetValue(_MGD, result) == RQ_SUCCESS)
		{
			if (result != lastTrackResult)
			{
				cout << (result ? "Track Present." : "Track Absent.") << endl;
			}
			lastTrackResult = result;
		}

		sleepms(100);
	}

	device.Disconnect();
	return 0;
}

int DisplaySamplesMenu()
{
	cout << "-----------------------------------------------" << endl;
	cout << "|                    ROBOTEQ                  |" << endl;
	cout << "-----------------------------------------------" << endl;
	cout <<  "Which sample would you like to run?" << endl;
	cout << "1. Motor Controller Sample." << endl;
	cout << "2. MagSensor Sample." << endl;
	cout << "3. Command Speed" << endl;
	cout << "Enter your choice: ";

	int choice;
	do
	{
		cin >> choice;
		if (choice <= 0 || choice > 2)
			cout << "Enter valid choice: ";
	} while (choice <=0 || choice > 3);

	return choice;
}

int main(int argc, char *argv[])
{


	string port = "/dev/ttyACM0";

	switch (DisplaySamplesMenu())
	{
	case 1:
		return MotorControllerSample(port);
	case 2:
		return MagSensorSample(port);
	case 3:
		return MotorControllerCommand(port, 2);
	}

	return 0;
	return MotorControllerSample(port);
}

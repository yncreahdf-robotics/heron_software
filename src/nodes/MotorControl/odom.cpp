/*
10/10/2019
author : Charles Goudaert	contact : charles.goudaert@isen.yncrea.fr
Based on the RoboteQ linux API
*/
#include <iostream>
#include <stdio.h>
#include <string.h>

#include "RobotSpecs.h"
#include "heron/Encoders.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;


class ProcessOdom
{
public:
    ProcessOdom()
    {
        sub = n.subscribe("sensor_enc", 100, callback);

        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    }


    void callback(const heron::Encoders& data)
    {
        rpm_frontLeft = data.EncFl;
        rpm_frontRight = data.EncFr;
        rpm_backLeft = data.EncBl;
        rpm_backRight = data.EncBr;

        //convert rpm into speeds
        vx = ((2*M_PI*WHEEL_RADIUS)/60) * (rpm_frontLeft + rpm_frontRight + rpm_backLeft + rpm_backRight)/4;		// m.s-1
        vy = ((2*M_PI*WHEEL_RADIUS)/60) * (- rpm_frontLeft + rpm_frontRight + rpm_backLeft - rpm_backRight)/4;	// m.s-1
        vth = 2*M_PI * (rpm_backRight - rpm_frontLeft) / (2*(WTOW_LENGHT + WTO_WIDTH));		// ras.s-1


        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).toSec();
        delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        delta_th = vth * dt;

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
    }

private:
    ros::NodeHandle n;
    ros::Publisher odom_pub;
    ros::Subscriber sub;

    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx;
    double vy;
    double vth;

    double dt;
    double delta_x;
    double delta_y;
    double delta_th;

    int rpm_frontLeft;
    int rpm_frontRight;
    int rpm_backLeft;
    int rpm_backRight;


};//End of class ProcessOdom




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom2");
	
    cout << "odom_Node Started" << endl;
    ProcessOdom POdom;

    ros::spin();
    
   return 0;
}



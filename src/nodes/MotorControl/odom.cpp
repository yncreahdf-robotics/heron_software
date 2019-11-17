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
private:
    ros::NodeHandle n;
    ros::Publisher odom_pub;
    ros::Subscriber sub;

    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;

    double x;
    double y;
    double th;

    double vx;
    double vy;
    double vth;

    double dt;
    double delta_x;
    double delta_y;
    double delta_th;

    double r_frontLeft;
    double r_frontRight;
    double r_backLeft;
    double r_backRight;


public:
    ProcessOdom()
    {
        cout << "Initialize Odom" << endl;
        sub = n.subscribe("sensor_encs", 10, &ProcessOdom::callback, this);
        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
        // set origin at 0;0;0
        x = 0;
        y = 0;
        th = 0;
    }
    ~ProcessOdom() {}


    void callback(const heron::Encoders& data)
    {
        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();

        // calculate the roatation made by each wheel
        r_frontLeft = -(data.EncFl / ENCODERS_COUNTABLE_EVENTS_OUTPUT_SHAFT) / dt;      // tr/s
        r_frontRight = -(data.EncFr / ENCODERS_COUNTABLE_EVENTS_OUTPUT_SHAFT) / dt;
        r_backLeft = -(data.EncBl / ENCODERS_COUNTABLE_EVENTS_OUTPUT_SHAFT) / dt;
        r_backRight = -(data.EncBr / ENCODERS_COUNTABLE_EVENTS_OUTPUT_SHAFT) / dt;

        vx = (2*M_PI*WHEEL_RADIUS) * (r_frontLeft + r_frontRight + r_backLeft + r_backRight)/4;        // m/s
        vy = (2*M_PI*WHEEL_RADIUS) * (- r_frontLeft + r_frontRight - r_backLeft + r_backRight)/4;      // m/s
        vth = - 2*M_PI*WHEEL_RADIUS * (+ r_frontLeft - r_frontRight - r_backLeft + r_backRight) / (4*(WTOW_LENGHT + WTO_WIDTH));                    // rad/s

        // debug
        // cout << endl << "odom vel : " << endl << "Vx: " << vx << " Vy: " << vy << " Vth: " << vth << endl;

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
        odom_trans.child_frame_id = "map";

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
        odom.child_frame_id = "map";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
    }


};//End of class ProcessOdom




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom2");
	
    ProcessOdom POdom;

    ros::spin();
    
   return 0;
}



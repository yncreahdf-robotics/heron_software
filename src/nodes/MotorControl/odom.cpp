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

    struct WheelsEncoders
    {
        int Fl, Fr, Bl, Br;
    };

    struct Pose
    {
        double x, y, th;
        double dt;
    };

    struct Speed
    {
        double vx, vy, vth;
    };

    WheelsEncoders diff_encs;

    Pose pose;
    Pose delta_poses;

    Speed speed;


public:
    ProcessOdom()
    {
        cout << "Initialize Odom" << endl;
        sub = n.subscribe("sensor_encs", 10, &ProcessOdom::callback, this);
        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

        // init map at 0;0;0
        pose.x = 0;
        pose.y = 0;
        pose.th = 0;
    }// End Constructor

    ~ProcessOdom() {}// End Destructor


    void callback(const heron::Encoders& data)
    {
        current_time = ros::Time::now();
        delta_poses.dt = (current_time - last_time).toSec();

        // calculate the rotation made by each wheel
        diff_encs.Fl = -(data.EncFl / ENCODERS_COUNTABLE_EVENTS_OUTPUT_SHAFT) / delta_poses.dt;      // tr/s
        diff_encs.Fr = -(data.EncFr / ENCODERS_COUNTABLE_EVENTS_OUTPUT_SHAFT) / delta_poses.dt;
        diff_encs.Bl = -(data.EncBl / ENCODERS_COUNTABLE_EVENTS_OUTPUT_SHAFT) / delta_poses.dt;
        diff_encs.Br = -(data.EncBr / ENCODERS_COUNTABLE_EVENTS_OUTPUT_SHAFT) / delta_poses.dt;

        speed.vx = (2*M_PI*WHEEL_RADIUS) * (diff_encs.Fl + diff_encs.Fr + diff_encs.Bl + diff_encs.Br)/4;        // m/s
        speed.vy = (2*M_PI*WHEEL_RADIUS) * (- diff_encs.Fl + diff_encs.Fr - diff_encs.Bl + diff_encs.Br)/4;      // m/s
        speed.vth = - 2*M_PI*WHEEL_RADIUS * (+ diff_encs.Fl - diff_encs.Fr - diff_encs.Bl + diff_encs.Br) / (4*(WTOW_LENGHT + WTO_WIDTH));   // rad/s

        if(speed.vx > MAX_SPEED+0.1 || speed.vy > MAX_SPEED+0.1)
        {
            ROS_INFO("Speed Jump detected");
            ROS_INFO("Speeds vx%f vy%f vth%f", speed.vx, speed.vy, speed.vth);
        }

        // debug
        // cout << endl << "odom vel : " << endl << "Vx: " << vx << " Vy: " << vy << " Vth: " << vth << endl;

        delta_poses.x = (speed.vx * cos(pose.th) - speed.vy * sin(pose.th)) * delta_poses.dt;
        delta_poses.y = (speed.vx * sin(pose.th) + speed.vy * cos(pose.th)) * delta_poses.dt;
        delta_poses.th = speed.vth * delta_poses.dt;

        // If delta poses are plosible, then update the pose, otherwise don't, it will publish previous ones
        if(delta_poses.x < MAX_DELTA_POSE && delta_poses.y < MAX_DELTA_POSE)
        {
            pose.x += delta_poses.x;
            pose.y += delta_poses.y;
            pose.th += delta_poses.th;
        }
        else
        {
            ROS_INFO("Pose Jump detected");
            ROS_INFO("Poses dx%f dy%f dth%f", delta_poses.x, delta_poses.y, delta_poses.th);
        }
        

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = pose.x;
        odom_trans.transform.translation.y = pose.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = pose.x;
        odom.pose.pose.position.y = pose.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = speed.vx;
        odom.twist.twist.linear.y = speed.vy;
        odom.twist.twist.angular.z = speed.vth;

        odom.pose.covariance[0] = 0.01;
        odom.pose.covariance[7] = 0.01;
        odom.pose.covariance[14] = 0.01;
        odom.pose.covariance[21] = 0.1;
        odom.pose.covariance[28] = 0.1;
        odom.pose.covariance[35] = 0.1;


        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
    }


};//End of class ProcessOdom




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom");
	
    ProcessOdom POdom;

    ros::spin();
    
   return 0;
}



#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "heron/winch.h"
#include <string.h>

using namespace std;
float plate_height;

void winchCallback(const heron::winch& msg) 
  {
    plate_height = (msg.height - 695) / 1000;
    //Test pour savoir le delta de la glissière
    //cout << plate_height << std::endl;
  };

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher_pieces");
  ros::NodeHandle n;
  ros::Subscriber sub; 

  tf::TransformBroadcaster base_broadcaster;
  tf::TransformBroadcaster plate_broadcaster;
  tf::TransformBroadcaster support_broadcaster;
  tf::TransformBroadcaster wheel_FR_broadcaster;
  tf::TransformBroadcaster wheel_FL_broadcaster;
  tf::TransformBroadcaster wheel_BR_broadcaster;
  tf::TransformBroadcaster wheel_BL_broadcaster;

  while(n.ok()){
    ros::spinOnce();

    sub = n.subscribe("winch_Height", 10, winchCallback);
    
    base_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.1)),
        ros::Time::now(),"odom", "base_link"));

    plate_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.32, 0, 0.13 + plate_height)),
        ros::Time::now(),"base_link", "plate"));
    
    support_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.3, 0, 0)),
        ros::Time::now(),"base_link", "support"));
    
    wheel_FR_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, -0.216235, -0.06)),
        ros::Time::now(),"base_link", "wheel_FR"));
    
    wheel_FL_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, 0.216235, -0.06)),
        ros::Time::now(),"base_link", "wheel_FL"));
    
    wheel_BR_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.2, -0.216235, -0.06)),
        ros::Time::now(),"base_link", "wheel_BR"));
    
    wheel_BL_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.2, 0.216235, -0.06)),
        ros::Time::now(),"base_link", "wheel_BL"));
  }
}
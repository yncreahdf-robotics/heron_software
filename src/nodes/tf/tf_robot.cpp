#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "heron/winch.h"
#include <sensor_msgs/JointState.h>
#define MAX_PLATE 1071.0
#define MIN_PLATE 695.0

using namespace std;
sensor_msgs::JointState joint_state;
ros::Publisher plate_pub;


void winchCallback(const heron::winch& msg) 
  {
    float plate_height;
    if (msg.height > MAX_PLATE)
    {
      plate_height = (MAX_PLATE - MIN_PLATE) / 1000;
    }
    else if (msg.height < MIN_PLATE)
    {
      plate_height = 0;
    }
    else
    {
      plate_height = (msg.height - MIN_PLATE) / 1000;
    }

    plate_pub.publish(joint_state);
    joint_state.position[0]=plate_height;
  };

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher_pieces");
  ros::NodeHandle n;
  ros::Subscriber sub;

  string tf_prefix;
  n.getParam("tf_prefix", tf_prefix);
  if(tf_prefix.size() > 0)
    {
      tf_prefix += "/";
    }

  joint_state.name.push_back("plate");
  joint_state.position.push_back(0);

  while(n.ok()){
    ros::spinOnce();

    sub = n.subscribe("winch_Height", 10, winchCallback);
    plate_pub = n.advertise<sensor_msgs::JointState>("plate_joint_states",10);

  }
}
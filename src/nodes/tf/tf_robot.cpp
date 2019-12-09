#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

void winchCallback(const ) 
  {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
  }

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_robot");
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
    sub = n.subscribe("winch_high", 10, winchCallback);

    base_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.1)),
        ros::Time::now(),"odom", "base_link"));

    plate_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.32, 0, 0.13)),
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
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster laser_broadcaster;
  tf::TransformBroadcaster imu_broadcaster;

  string tf_prefix;
  n.getParam("tf_prefix", tf_prefix);
  if(tf_prefix.size() > 0)
    {
      tf_prefix += "/";
    }

  tf::Quaternion laser_quat;
  laser_quat.setRPY(0, 0, M_PI);

  tf::Quaternion imu_quat;
  imu_quat.setRPY(0, 0, M_PI);

  while(n.ok()){
    laser_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(laser_quat, tf::Vector3(0.0, 0.0, 0.2)),
        ros::Time::now(),tf_prefix + "base_link", tf_prefix + "laser"));

    imu_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(imu_quat, tf::Vector3(0.0, 0.0, 0.09)),
        ros::Time::now(), tf_prefix + "base_link", tf_prefix + "imu"));


    r.sleep();
  }
}
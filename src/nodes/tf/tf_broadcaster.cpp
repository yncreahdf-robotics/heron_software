#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster laser_broadcaster;
  tf::TransformBroadcaster imu_broadcaster;

  tf::Quaternion laser_quat;
  laser_quat.setRPY(0, 0, M_PI);

  tf::Quaternion imu_quat;
  imu_quat.setRPY(0, 0, M_PI);

  while(n.ok()){
    laser_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(laser_quat, tf::Vector3(0.0, 0.0, 0.2)),
        ros::Time::now(),"base_link", "laser"));

    imu_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(imu_quat, tf::Vector3(0.0, 0.0, 0.09)),
        ros::Time::now(),"base_link", "imu"));


    r.sleep();
  }
}
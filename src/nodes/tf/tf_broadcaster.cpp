#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster base_broadcaster;
  tf::TransformBroadcaster plate_broadcaster;
  tf::TransformBroadcaster laser_broadcaster;
  tf::TransformBroadcaster imu_broadcaster;

  while(n.ok()){
    base_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.1)),
        ros::Time::now(),"odom", "base_link"));

    plate_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"base_link", "plate"));

    laser_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.2)),
        ros::Time::now(),"base_link", "laser"));

    imu_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.09)),
        ros::Time::now(),"base_link", "imu"));


    r.sleep();
  }
}
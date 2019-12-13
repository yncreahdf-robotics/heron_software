#include <iostream>
#include <stdio.h>
#include <string.h>

#include "heron/Motion.h"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Float32.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Motion
{
    private:
        
        ros::NodeHandle n;

		ros::Subscriber sub;
        ros::Publisher winch_pub;

        // custom msg
        heron::Motion motion_msg;

        string tf_prefix;


    public:
        Motion()
        {
            cout << endl << "Initialize Move to Action" << endl;
            //Subscribe to move_to topic and call the moveTo function 
            sub = n.subscribe("move_to", 1, &Motion::moveTo, this);
            winch_pub = n.advertise<std_msgs::Float32>("cmd_pos_winch", 1);

            n.getParam("tf_prefix", tf_prefix);
            if(tf_prefix.size() > 0)
            {
                tf_prefix += "/";
            }

        }// End Constructor

        ~Motion() {}// End Destructor

        void moveTo(const heron::Motion& data)
        {
            //tell the action client that we want to spin a thread by default
            MoveBaseClient ac("move_base", true);

            //wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
            }

            move_base_msgs::MoveBaseGoal goal;

            std_msgs::Float32 winch_pos;

            //we'll send a goal to the robot to move 
            goal.target_pose.header.frame_id = tf_prefix + "base_link";
            goal.target_pose.header.stamp = ros::Time::now();

            // put the plate down before moving
            winch_pos.data = 0;
            winch_pub.publish(winch_pos);

            goal.target_pose.pose.position.x = data.position_x;
            goal.target_pose.pose.position.y = data.position_y;
            goal.target_pose.pose.orientation.z = data.orientation_z;
            goal.target_pose.pose.orientation.w = data.orientation_w;

            
            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                // once it reaches the position we get the plate leveled
                winch_pos.data = data.plate_height;
                winch_pub.publish(winch_pos);
                ROS_INFO("Hooray, the base moved to the location");
            else
                ROS_INFO("The base failed to move to the location for some reason");
        }
};









int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");

    Motion motion();

    ros::spin();
    
    return 0;
  
}
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import os, sys
import roslaunch


controller_switch = False
flag = False

def callback(data):
    global controller_switch
    global flag
    # left menu button
    if(data.buttons[6]):
        controller_switch = not(controller_switch)
        flag = True


if __name__ == '__main__':
    # starts the node
    rospy.init_node('remote', log_level = rospy.INFO)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)

    while not rospy.is_shutdown():

        if(flag):

            # start xboxController to pilot the robot
            if(controller_switch):
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                controller_launch = roslaunch.parent.ROSLaunchParent(uuid,[str(os.getcwd())+"/catkin_ws/src/heron_software/src/launch/xboxController.launch"])

                controller_launch.start()
                rospy.loginfo("launching controller")

            else:
                controller_launch.shutdown()
                rospy.loginfo("shutting down controller")        

            flag = False

        rospy.sleep(1)
    

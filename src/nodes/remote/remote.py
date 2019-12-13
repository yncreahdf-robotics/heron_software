#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import os, sys
import roslaunch


switch = False
flag = False

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid,[str(os.getcwd())+"/catkin_ws/src/heron_software/src/launch/xboxController.launch"])

def callback(data):
    global switch
    global flag
    if(data.buttons[6]):
        switch = not(switch)
        flag = True


if __name__ == '__main__':
    # starts the node
    rospy.init_node('remote')

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    progstop = True
    while not rospy.is_shutdown() and progstop:

        if(flag):
            if(switch):
                launch.start()
                print("launching controller")

            else:
                launch.shutdown()
                print("shutting down controller")
                progstop = False
        
            flag = False
        rospy.sleep(1)
    

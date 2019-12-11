#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import os, sys
import roslaunch


switch = True

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid,[str(os.getcwd())+"/catkin_ws/src/heron_software/src/launch/xboxController.launch"])

def dummy_function(): pass
launch._init_signal_handlers = dummy_function

def callback(data):
    global switch
    if(data.buttons[6]):
        switch = not(switch)

        if(switch):
            os.system("roslaunch heron xboxController.launch")
            launch.start()
            print("launching controller")

        if(not(switch)):
            launch.shutdown()
            print("shutting down controller")

    

if __name__ == '__main__':
    # starts the node
    rospy.init_node('remote')

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)

    rospy.spin()
    

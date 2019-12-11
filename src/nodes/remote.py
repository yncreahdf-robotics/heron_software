#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import os, sys
import roslaunch

switch = False

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid,[str(os.getcwd())+"/catkin_ws/src/heron_software/src/launch/xboxController.launch"])

def callback(data):
    global switch
    global launch
    if(data.buttons[6]):
        switch = not(switch)

        if(switch):
            try:
                launch.start()
                launch.
                print("launching controller")
            except:
                print("launching failed")
        else:
            launch.shutdown()
            print("shutting down controller")




# Intializes everything
def start():
     # starts the node
    rospy.init_node('remote')

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)   
    rospy.spin()

if __name__ == '__main__':
    start()
    

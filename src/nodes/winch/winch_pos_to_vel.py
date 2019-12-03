#!/usr/bin/env python

import winch_specs as wch
import diagnostic_msgs
import roboclaw_driver as roboclaw
from std_msgs.msg import Int16,Float32 
from heron.msg import winch as MsgWinch
import rospy
import os

import sys   
sys.path.append(str(os.getcwd()[:-4])+"catkin_ws/src/heron_software/src/nodes/winch") 
import winch_specs


pub = rospy.Publisher('cmd_vel_winch', Float32, queue_size=10)


#height = winch_specs.MINHEIGHT
#heightTicks = 0

def update_height(data):
    global heightTicks
    global height
    height=data.height
    heightTicks=data.heightTicks
    #rospy.loginfo("height recue %f", height)
    #rospy.loginfo("heightTicks recue %f", heightTicks)

def callback(data):
    global height
    desired_height = data.data
    current_height = height
    rospy.loginfo("pos desired %d     actual pos %d",desired_height,current_height )

    if ((desired_height < winch_specs.MINHEIGHT) or (desired_height > winch_specs.MAXHEIGHT-1)):
        rospy.logwarn("Pos out of range")

    
    # if we need to back speed
    if (current_height > desired_height):
        while (current_height > desired_height):
            pub.publish(-winch_specs.MAXSPEED_M_S) 
            rospy.logdebug("going backward")

    elif (current_height < desired_height):
        while (current_height < desired_height):
            pub.publish(winch_specs.MAXSPEED_M_S) 
            rospy.logdebug("going forward")
        
    else:
        pub.publish(0)
        rospy.logdebug("Pos reached")

        
    


def start():
    rospy.init_node("winch_node_pos_to_vel",log_level=rospy.DEBUG)
    
    rospy.Subscriber("cmd_pos_winch", Float32, callback, queue_size = 1)
    rospy.Subscriber("winch_Height", MsgWinch , update_height, queue_size = 5)

    
   
   

if __name__ == '__main__':
    start()
    rospy.spin()
   
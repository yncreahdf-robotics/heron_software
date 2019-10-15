#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import math
import numpy as np

# This ROS Node converts Joystick inputs from the joy node
# into commands for Heron

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()
    twist.linear.x = data.axes[1] * 0.44 #robot linear speed
    twist.linear.y = data.axes[0] * 0.44

    if(data.axes[3] == 0 and data.axes[4] == 0):
        twist.angular.z = 0
    else:
        twist.angular.z = (math.atan2(-data.axes[3], -data.axes[4]) + np.pi) # 3;4

    if(data.buttons[3]):
        twist.linear.z = 1
    if(data.buttons[1]):
        twist.linear.z = 2
    if(data.buttons[0]):
        twist.linear.z = 3
    if(data.buttons[2]):
        twist.linear.z = 4
    
    print("x: ", twist.linear.x, " y: ", twist.linear.y, " a: ", twist.angular.z * 360 / (2*np.pi))
    pub.publish(twist)


# Intializes everything
def start():
    # publishing to "Heron/cmd_vel" to control Heron
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)


    # starts the node
    rospy.init_node('controller')
    rospy.spin()

if __name__ == '__main__':
    start()
    

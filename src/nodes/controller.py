#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from math import pi, atan2, sqrt


# This ROS Node converts Joystick inputs from the joy node
# into commands for Heron

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls linear speed
# axis 3 aka right stick horizonal controls angular speed
def callback(data):
    twist = Twist()
    twist.linear.x = data.axes[1] * 0.40 #robot linear speed
    twist.linear.y = data.axes[0] * 0.40

    # if(data.axes[3] == 0 and data.axes[4] == 0):
    #     twist.angular.z = 0
    # else:
    #     twist.angular.z = (atan2(-data.axes[3], -data.axes[4]) + pi) # 3;4

    twist.angular.z = data.axes[3] * pi/2


    if(data.axes[7] < 0):
        twist.linear.x = 0.44
    if(data.axes[7] > 0):
        twist.linear.x = -0.44
    if(data.axes[6] < 0):
        twist.linear.y = -0.44
    if(data.axes[6] > 0):
        twist.linear.y = 0.44
    
    # if(data.axes[2] > 0 or data.axes[5] > 0):
    #     twist.linear.x = 0
    #     twist.linear.y = 0
    #     twist.angular.x = 0
    
    print('x: ', twist.linear.x, ' y: ', twist.linear.y, ' a: ', twist.angular.z * 360 / (2*pi))
    pub.publish(twist)


# Intializes everything
def start():
    # publishing to "Heron/cmd_vel" to control Heron
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)


    # starts the node
    rospy.init_node('controller')
    rospy.spin()

if __name__ == '__main__':
    start()
    

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# This ROS Node converts Joystick inputs from the joy node
# into commands for Heron

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()
    twist.linear.x = data.axes[1]*1000
    twist.linear.y = data.axes[0]*1000

    twist.angular.z = data.axes[3]*1000
    
    print("callback")
    pub.publish(twist)


# Intializes everything
def start():
    # publishing to "Heron/cmd_vel" to control Heron
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)


    # starts the node
    rospy.init_node('controller')
    rospy.spin()

if __name__ == '__main__':
    start()
    

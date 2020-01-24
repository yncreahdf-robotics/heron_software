#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped

positionMsg = str()
index = 0

def callback(data):
    global index
    # left menu button
    if(data.buttons[6]):
        file = open("../../../map/positions.txt", "a")
        file.write("position " + str(index) + " ")
        file.write(positionMsg)
        file.close()
        index += 1

def getPos(data):
    global positionMsg
    positionMsg = str(data.pose.pose.position.x) + ";" + str(data.pose.pose.position.y) + ";" + str(data.pose.pose.orientation.z) + ";" + str(data.pose.pose.orientation.w) + "/n"


if __name__ == '__main__':
    # starts the node
    rospy.init_node('remote')

    file = open("positions.txt", "w")
    file.write("Next follows the positions registered in the same order than recorded \n")
    file.close()

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)

    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, getPos)

    rospy.spin()

    
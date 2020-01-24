#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped
from heron.msg import winch as MsgWinch

positionMsg = str()
heightMsg = str()
index = 0
tmp_pos = str()

def callback(data):
    global index
    global tmp_pos 
    # left menu button
    if(data.buttons[0]):
        if(tmp_pos != positionMsg):
            file = open("../MapKeyPos.txt", "a")
            file.write("position " + str(index) + " ")
            file.write(positionMsg)
            print("writting Position" + str(index) + "into the file.")
            file.close()
            index += 1
        tmp_pos = positionMsg

def getPos(data):
    global positionMsg
    positionMsg = str(data.pose.pose.position.x) + ";" + str(data.pose.pose.position.y) + ";" + str(data.pose.pose.orientation.z) + ";" + str(data.pose.pose.orientation.w) + ";" + heightMsg + "\n"

def getHeight(data):
    global heightMsg
    heightMsg = str(data.height)

if __name__ == '__main__':
    file = open("../MapKeyPos.txt", "w")
    file.write("Next follows the positions on the map registered in the same order than recorded \n")
    file.close()
    # starts the node
    rospy.init_node('remote')

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)

    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, getPos)

    rospy.Subscriber("winch_Height", MsgWinch, getHeight)

    rospy.spin()

    
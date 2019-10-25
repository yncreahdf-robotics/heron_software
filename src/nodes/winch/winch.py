#!/usr/bin/env python
import rospy
from roboclaw_3 import Roboclaw
from std_msgs.msg import Int16

#Windows comport name
#rc = Roboclaw("COM14",115200)
#Linux comport name
rc = Roboclaw("/dev/ttyACM0",115200)
 

address = 0x80
minHeight = 69 #cm
maxHeight = 108 #cm

def displayspeed():
    enc = rc.ReadEncM1(address)
    speed = rc.ReadSpeedM1(address)

    if(enc[0]==1):
        print ("Encoder:",enc[1])
    else:
        print ("encoder failed")

    if(speed[0]):
        print("speed cm/s: ",round((speed[1]*15.88)/2775,2))
        print ("speed: ",speed[1])
    else:
        print ("speed failed")



#QPPS is the speed of the encoder when the motor is at 100% power
#adresse 65 -> SpeedAccelDeccelPositionM1(adress, accel, speed, deccel, position, buffer)
#For an accel=speed in QPPS, the speed is reached in 1 sec
#For an accel=0,5speed in QPPS, the speed is reached in 2sec
# Our motor is 1150 RPM, there are 145.6 tick of encoder per revolution.
# we have an engrenage 24:1 ratio, so 1 output revolution = 24*145.6 = 3494.4 ticks
# 1cm = 183,92 ticks (our output cylinder is 19cm perimeter)
#2775 tick/sec max speed -> 15.88cm/sec max speed
#max height robot 108cm min height 69cm -> 39cm of cable

# max 7172,88
def callback(height):

    if (height < minHeight or  height > maxHeight):
        return 0
    else:
        height-= minHeight
        goal = int(183.92*height)
        print("Position: ",goal)
        rc.SpeedAccelDeccelPositionM1(address,1000,1000,1000,goal,0)

        while(rc.ReadEncM1(address)[1]!= goal):
            displayspeed()
            
        print("fin")

    pub.publish(rc.ReadEncM1(address)[1])

def start():
    # publishing to "Heron/cmd_vel" to control Heron
     # starts the node
    rospy.init_node('winch')

    global pub
    pub = rospy.Publisher('winch_Height', Int16, queue_size=50)
    
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("cmd_winch", Int16, callback)

    if (rc.Open()==0):
        rospy.loginfo("connexion failed")
    else : 
        rospy.loginfo("connexion ok") 
        rospy.spin()

if __name__ == '__main__':
    start()
    

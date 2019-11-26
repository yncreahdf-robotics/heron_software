#!/usr/bin/env python

from math import pi, cos, sin

import diagnostic_msgs
import roboclaw_driver as roboclaw
from std_msgs.msg import Int16,Float32 
import rospy


# QPPS is the speed of the encoder when the motor is at 100% power
# adresse 65 -> SpeedAccelDeccelPositionM1(adress, accel, speed, deccel, position, buffer)
# For an accel=speed in QPPS, the speed is reached in 1 sec
# For an accel=0,5speed in QPPS, the speed is reached in 2sec
# Our motor is 1150 RPM max, there are 145.6 ticks of encoder per revolution.
# We have an engrenage 24:1 ratio, so 1 output revolution = 24*145.6 = 3494.4ticks
# 1mm = 37.715 ticks (386 mm correspond to 14558 ticks)
# 2775 tick/sec max speed (blocked at 1500) -> 0.039772 m/sec max speed
# max height robot 1076 mm    min height 690 mm -> 390 mm delta
# max 14558 ticks to reach 1076 mm height
# 0 tick -> 690 mm height
# 14558 ticks --> 1076 mm height

address = 0x80

MINHEIGHT = 690 #mm
MAXHEIGHT = 1076 #mm

MAXSPEEDTICKS = 1500 #ticks
MAXTICKS = 14558 #ticks

TICKSPERMM = MAXTICKS/(MAXHEIGHT-MINHEIGHT) #ticks for 1 mm
MAXSPEED_M_S = (MAXSPEEDTICKS/TICKSPERMM)*0.001 # m.s-1

ACCELTICKS = 800 #ticks
DECELTICKS = 800 #ticks


pub = rospy.Publisher('winch_Height', Float32, queue_size=50)

def displayspeed():
    enc = roboclaw.ReadEncM1(address)
    speed = roboclaw.ReadSpeedM1(address)

    if(enc[0]==1):
        rospy.logdebug("Encoder: %d",enc[1])
    else:
        rospy.logerr("encoder failed")

    if(speed[0]):
        rospy.logdebug("speed mm/s: %f ",round((speed[1]*9.705)/MAXSPEEDTICKS,2))
        rospy.logdebug ("speed: %d",speed[1])
    else:
        rospy.logerr("speed failed")


def calculateHeight():
    heihtTicks = roboclaw.ReadEncM1(address)[1]
    height = round(MINHEIGHT + heihtTicks/TICKSPERMM,2)
    return(height, heightTicks)

def controllerInput(data):
    desiredSpeedInMS = data.data #speed input from remote controller 
    desiredSpeedInTicks = data.data*1000*TICKSPERMM
    rospy.loginfo("Desired Speed m.s: %f",speedMS)

    realSpeed = roboclaw.ReadSpeedM1(address)
    realSpeedInMS = int((realSpeed[1]*MAXSPEED_M_S)/MAXSPEEDTICKS,2)

    height = calculateHeight()[1]

    if (desiredSpeedInMS < -MAXSPEEDTICKS or desiredSpeedInMS > MAXSPEEDTICKS):
        rospy.logerr("Speed out of range")
    if (height < 0 or height > MAXTICKS):
        rospy.logerr("Position out of range, shutdown motor")
        roboclaw.SpeedAccelM1(address,1500,0)
    else: 
        roboclaw.SpeedAccelM1(address,800,desiredSpeedInMS)





def callback(data):

    height = data.data  #value received from subscriber
    rospy.loginfo("height : %d",height)

    if (height < MINHEIGHT or  height > MAXHEIGHT):
        rospy.logerr("Wrong distance")
        
    else:
        height-= MINHEIGHT
        goal = int(TICKSPERMM*height)
        enc = roboclaw.ReadEncM1(address)[1]
        rospy.logdebug("Position goal: %d  Actual Pos: %d",goal,enc)
        
        roboclaw.SpeedAccelDeccelPositionM1(address,ACCELTICKS,MAXSPEEDTICKS,DECELTICKS,goal,0)

        while(roboclaw.ReadEncM1(address)[1]!= goal):
            statut = roboclaw.ReadError(address)[1]
            #rospy.logdebug("statut : %s",statut)
            if  (statut == 0x4000) :
                rospy.logdebug("M1 Home reached")
                roboclaw.ResetEncoders(address)
            elif (statut == 0x8000):
                maxTicks = roboclaw.ReadEncM1(address)[1]
                rospy.logdebug("M1 Home reached  pos : %d",maxTicks)
                roboclaw.SetEncM1(address,maxTicks)
                roboclaw.SpeedAccelM1(address,1500,0)
            pub.publish(calculateHeight()[0])
            #displayspeed()
        #calculateHeight()
        enc = roboclaw.ReadEncM1(address)[1]
        rospy.logdebug("Position goal: %d  Actual Pos: %d",goal,enc)
        rospy.loginfo("End")

   #pub.publish(calculateHeight())


def shutdown():
        rospy.loginfo("Shutting down")
        try:
            roboclaw.SpeedAccelM1(address,1500, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.SpeedAccelM1(address,1500, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)

def start():
    # publishing to "Heron/cmd_vel" to control Heron
    # starts the node
    rospy.init_node("winch_node",log_level=rospy.DEBUG)
    
    rospy.Subscriber("cmd_winch", Float32, callback)

    rospy.on_shutdown(shutdown)
    rospy.loginfo("Connecting to roboclaw")

    dev_name = rospy.get_param("~dev", "/dev/winch")
    baud_rate = int(rospy.get_param("~baud", "115200"))

    try:
        roboclaw.Open(dev_name, baud_rate)
    except Exception as e:
        rospy.logfatal("Could not connect to Roboclaw")
        rospy.logdebug(e)
        rospy.signal_shutdown("Could not connect to Roboclaw")
      
    try:
        version = roboclaw.ReadVersion(address)
    except Exception as e:
        rospy.logwarn("Problem getting roboclaw version")
        rospy.logdebug(e)
        pass

    if not version[0]:
        rospy.logwarn("Could not get version from roboclaw")
    else:
        rospy.logdebug(repr(version[1]))


    roboclaw.SpeedAccelM1(address,1500, 0)
    #roboclaw.ResetEncoders(address)

    rospy.logdebug("dev %s", dev_name)
    rospy.logdebug("baud %d", baud_rate)
    rospy.logdebug("address %d", address)
   
   
    

if __name__ == '__main__':
   
    start()
    rospy.spin()
    

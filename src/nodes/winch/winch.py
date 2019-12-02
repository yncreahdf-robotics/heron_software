#!/usr/bin/env python

from math import pi, cos, sin

import winch_specs as wch
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

pub = rospy.Publisher('winch_Height', Float32, queue_size=50)



def calculateHeight():
    heightTicks = roboclaw.ReadEncM1(address)[1]
    height = wch.MINHEIGHT + heightTicks/wch.TICKSPERMM
    return(height, heightTicks)


def controllerInput(data):
    desiredSpeedInMS = data.data #speed input from remote controller m.s-1
    desiredSpeedInTicks = data.data*1000*wch.TICKSPERMM # speed from m.s-1 to ticks/sec
    rospy.loginfo("Speed m.s: %f  Speed in ticks : %f  ",desiredSpeedInMS, desiredSpeedInTicks)

    realSpeed = roboclaw.ReadSpeedM1(address)
    realSpeedInMS = (realSpeed[1]*wch.MAXSPEED_M_S)/wch.MAXSPEEDTICKS
    rospy.loginfo("real  m.s: %f  real ticks     : %f  ",realSpeedInMS, realSpeed[1])

    heightData = calculateHeight()
    height = heightData[1] #in ticks
    
    #rospy.loginfo("height %f:",height)

    statut=roboclaw.ReadError(address)[1]
    
    if  (statut == 4194304) :  #0x40
        rospy.logdebug("M1 Home reached")
    elif (statut == 8388608):  #0x80
        rospy.logdebug("M1 Max Pos reached")
        #roboclaw.SetEncM1(address,maxTicks)

    if (desiredSpeedInTicks < -wch.MAXSPEEDTICKS or desiredSpeedInTicks > wch.MAXSPEEDTICKS):
        rospy.logerr("Speed out of range")
        roboclaw.SpeedAccelM1(address,80000,0)

    if (height < 100):  
        if(desiredSpeedInTicks < 0):
            roboclaw.SpeedAccelM1(address,80000,0)
        else:
            roboclaw.SpeedAccelM1(address,wch.ACCELTICKS,int(desiredSpeedInTicks))
        

    elif (height > (wch.MAXTICKS-100)):
        if(desiredSpeedInTicks > 0):
            roboclaw.SpeedAccelM1(address,80000,0)
        else:
            roboclaw.SpeedAccelM1(address,wch.ACCELTICKS,int(desiredSpeedInTicks))
        
   
    else: 
        roboclaw.SpeedAccelM1(address,wch.ACCELTICKS,int(desiredSpeedInTicks))

    pub.publish(heightData[0])
     
        

    





def callback(data):

    height = data.data  #value received from subscriber
    rospy.loginfo("height : %d",height)

    if (height < wch.MINHEIGHT or  height > wch.MAXHEIGHT):
        rospy.logerr("Wrong distance")
        
    else:
        height-= wch.MINHEIGHT
        goal = int(wch.TICKSPERMM*height)
        enc = roboclaw.ReadEncM1(address)[1]
        rospy.logdebug("Position goal: %d  Actual Pos: %d",goal,enc)
        
        roboclaw.SpeedAccelDeccelPositionM1(address,wch.ACCELTICKS,wch.MAXSPEEDTICKS,wch.DECELTICKS,goal,0)

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
        enc = roboclaw.ReadEncM1(address)[1]
        rospy.logdebug("Position goal: %d  Actual Pos: %d",goal,enc)
        rospy.loginfo("End")



def shutdown():
        rospy.loginfo("Shutting down")
        try:
            roboclaw.SpeedAccelM1(address,80000, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.SpeedAccelM1(address,80000, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)

def start():
    # publishing to "Heron/cmd_vel" to control Heron
    # starts the node
    rospy.init_node("winch_node",log_level=rospy.DEBUG)
    
    rospy.Subscriber("cmd_pos_winch", Float32, callback,queue_size=1)
    rospy.Subscriber("cmd_vel_Winch",Float32, controllerInput, queue_size=1)

    rospy.on_shutdown(shutdown)
    rospy.loginfo("Connecting to roboclaw")

    dev_name = rospy.get_param("~dev", "/dev/winch")
    baud_rate = int(rospy.get_param("~baud", "115200"))

    try:
        roboclaw.Open(dev_name, baud_rate)
        return 1
    except Exception as e:
        rospy.logfatal("Could not connect to Roboclaw")
        rospy.logdebug(e)
        rospy.signal_shutdown("Could not connect to Roboclaw")
        return 0
      
    roboclaw.SpeedAccelM1(address,80000, 0)
    roboclaw.ResetEncoders(address)

    rospy.logdebug("dev %s", dev_name)
    rospy.logdebug("baud %d", baud_rate)
    rospy.logdebug("address %d", address)
   
   
    

if __name__ == '__main__':
    start()
    rospy.spin()
    
    # if (start()):
    #     rospy.spin()
    # else : 
    #     return 0 #could not connect 
    
    

#!/usr/bin/env python

import winch_specs as wch
import diagnostic_msgs
import roboclaw_driver as roboclaw
from std_msgs.msg import Int16,Float32 
from heron.msg import winch as MsgWinch
import rospy
import sys


# QPPS is the speed of the encoder when the motor is at 100% power
# adresse 65 -> SpeedAccelDeccelPositionM1(adress, accel, speed, deccel, position, buffer)
# For an accel=speed in QPPS, the speed is reached in 1 sec
# For an accel=0,5speed in QPPS, the speed is reached in 2sec
# Our motor is 1150 RPM max, there are 145.6 ticks of encoder per revolution.
# We have an engrenage 24:1 ratio, so 1 output revolution = 24*145.6 = 3494.4ticks
# 1mm = 38.1117 ticks (376 mm correspond to 14330 ticks)
# 2775 tick/sec max speed (blocked at 1700) -> 0.0446 m/sec max speed
# max height robot 1071 mm    min height 695 mm -> 376 mm delta
# max 14330 ticks to reach 1071 mm height
# 0 tick -> 695 mm height
# 14330 ticks --> 1071 mm height

address = 0x80

pub = rospy.Publisher('winch_Height', MsgWinch, queue_size=10)


def posInput(data):
    winchData = MsgWinch() #msg declaration

    heightDesired_mm = data.data*1000 #we receive msg in meters, we work in mm
    
    if (heightDesired_mm < wch.MINHEIGHT or  heightDesired_mm > wch.MAXHEIGHT-1):
        rospy.logerr("Wrong Position")

        heightData = calculateHeight()
        heightTicks = heightData[1] #in ticks
        heightmm = heightData[0]

        winchData.height = heightmm/1000 # in meters
        winchData.heightTicks = heightTicks 
        pub.publish(winchData)
    
    else:
        desiredPos = int((heightDesired_mm-wch.MINHEIGHT)*wch.TICKSPERMM)
        rospy.loginfo("Position goal: %d ",desiredPos)
        roboclaw.SpeedAccelDeccelPositionM1(address,wch.ACCELTICKS,wch.MAXSPEEDTICKS,wch.DECELTICKS,desiredPos,0)

        while(roboclaw.ReadEncM1(address)[1]!= desiredPos):
            heightData = calculateHeight()
            heightTicks = heightData[1] #in ticks
            heightmm = heightData[0]

            winchData.height = heightmm/1000 # in meters
            winchData.heightTicks = heightTicks 
            pub.publish(winchData)
            
        rospy.loginfo("Pos reached")

    pub.publish(winchData)


def calculateHeight():
    heightTicks = roboclaw.ReadEncM1(address)[1]
    height = wch.MINHEIGHT + heightTicks/wch.TICKSPERMM
    return(height, heightTicks)



def controllerInput(data):
    winchData=MsgWinch()

    desiredSpeedInMS = data.data #speed input from remote controller m.s-1
    desiredSpeedInTicks = data.data*1000*wch.TICKSPERMM # speed from m.s-1 to ticks/sec

    rospy.logdebug("Speed m.s: %f  Speed in ticks : %f  ",desiredSpeedInMS, desiredSpeedInTicks)
    realSpeed = roboclaw.ReadSpeedM1(address)
    realSpeedInMS = (realSpeed[1]*wch.MAXSPEED_M_S)/wch.MAXSPEEDTICKS
    rospy.logdebug("real  m.s: %f  real ticks     : %f  ",realSpeedInMS, realSpeed[1])

    heightData = calculateHeight()
    heightTicks = heightData[1] #in ticks
    
    statut=roboclaw.ReadError(address)[1]
    if  (statut == 4194304) :  #0x40 driver code
        rospy.logdebug("M1 Home reached")
    elif (statut == 8388608):  #0x80
        rospy.logdebug("M1 Max Pos reached")
        #roboclaw.SetEncM1(address,maxTicks)

    if (desiredSpeedInTicks < -wch.MAXSPEEDTICKS or desiredSpeedInTicks > wch.MAXSPEEDTICKS):
        rospy.logerr("Speed out of range")
        roboclaw.SpeedAccelM1(address,80000,0)

    #checks if winch is near to bottom
    if (heightTicks < 100):  
        if(desiredSpeedInTicks < 0):
            roboclaw.SpeedAccelM1(address,80000,0)
        else:
            roboclaw.SpeedAccelM1(address,wch.ACCELTICKS,int(desiredSpeedInTicks))

    #checks if winch is near to top 
    elif (heightTicks > (wch.MAXTICKS-100)):
        if(desiredSpeedInTicks > 0):
            roboclaw.SpeedAccelM1(address,80000,0)
        else:
            roboclaw.SpeedAccelM1(address,wch.ACCELTICKS,int(desiredSpeedInTicks))
        
    #if winch is between bot and top
    else: 
        roboclaw.SpeedAccelM1(address,wch.ACCELTICKS,int(desiredSpeedInTicks))
    
    heightData = calculateHeight()
    heightTicks = heightData[1] #in ticks
    heightmm = heightData[0]

    winchData.height = heightmm/1000 # in meters
    winchData.heightTicks = heightTicks 
    pub.publish(winchData)
    
     

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
    rospy.init_node("winch_node",log_level=rospy.INFO)
    
    rospy.Subscriber("cmd_vel_winch", Float32, controllerInput, queue_size=1)
    rospy.Subscriber("cmd_pos_winch", Float32, posInput, queue_size = 1)  # height desired in meter

    

    rospy.on_shutdown(shutdown)
    rospy.loginfo("Connecting to roboclaw")

    dev_name = rospy.get_param("~dev", "/dev/winch")
    baud_rate = int(rospy.get_param("~baud", "115200"))

    if(roboclaw.Open(dev_name, baud_rate)):
        rospy.loginfo("Connected to roboclaw")
        roboclaw.SpeedAccelM1(address,80000, 0)
        roboclaw.ResetEncoders(address)
        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address %d", address)
        return 1

    else:
        rospy.logerr("Could not open Port !")
        rospy.signal_shutdown("Could not connect to Roboclaw")
        return 0

    
      
    
    

if __name__ == '__main__':
   
    if (start()):
        rospy.spin()
    else : 
        rospy.logerr("Error can't initialize node")
        sys.exit(0) #could not connect 
    

    

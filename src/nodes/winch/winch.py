#!/usr/bin/env python

from math import pi, cos, sin

import winch_specs as wch
import diagnostic_msgs
import roboclaw_driver as roboclaw
from std_msgs.msg import Int16,Float32 
import rospy
import threading



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

def publisher_thread():
    rate=rospy.Rate(100)
    while not rospy.is_shutdown():
        global height
        heightTicks = roboclaw.ReadEncM1(address)[1]
        height = wch.MINHEIGHT + heightTicks/wch.TICKSPERMM
        pub.publish(heightTicks)
        rate.sleep()




def controllerInput(data):
    desiredSpeedInMS = data.data #speed input from remote controller m.s-1
    desiredSpeedInTicks = data.data*1000*wch.TICKSPERMM # speed in ticks/sec
    rospy.loginfo("Speed m.s: %f  Speed in ticks : %f  ",desiredSpeedInMS, desiredSpeedInTicks)

    realSpeed = roboclaw.ReadSpeedM1(address)
    realSpeedInMS = (realSpeed[1]*wch.MAXSPEED_M_S)/wch.MAXSPEEDTICKS
    rospy.loginfo("real  m.s: %f  real ticks     : %f  ",realSpeedInMS, realSpeed[1])

    heightData = calculateHeight()
    height = heightData[1] #in ticks
    
    #rospy.loginfo(" ")
    rospy.loginfo("height %f:",height)

    if (desiredSpeedInTicks < -wch.MAXSPEEDTICKS or desiredSpeedInTicks > wch.MAXSPEEDTICKS):
        rospy.logerr("Speed out of range")
        roboclaw.SpeedAccelM1(address,1500,0)


   

    if (height < 800):
        roboclaw.SpeedAccelM1(address,1500,int(desiredSpeedInTicks*0.1))

    elif (height < 0):    
        roboclaw.SpeedAccelM1(address,1500, 500)

    # if (height > wch.MAXTICKS-600):
    #     roboclaw.SpeedAccelM1(address,1500,int(desiredSpeedInTicks*0.08))

    # if (height > wch.MAXTICKS):
    #     roboclaw.SpeedAccelM1(address,1500,500)
   
    else: 
        roboclaw.SpeedAccelM1(address,1500,int(desiredSpeedInTicks))
        #roboclaw.SpeedM1(address,800)
        #roboclaw.SpeedM1(address,1500)
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
    
    rospy.Subscriber("cmd_winch", Float32, callback,queue_size=1)
    rospy.Subscriber("cmd_vel_Winch",Float32, controllerInput, queue_size=1)

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
    roboclaw.ResetEncoders(address)

    rospy.logdebug("dev %s", dev_name)
    rospy.logdebug("baud %d", baud_rate)
    rospy.logdebug("address %d", address)
   
   
    

if __name__ == '__main__':
    
    start()
    #worker = threading.Thread(target=publisher_thread)
    #worker.start()
    
    rospy.spin()

    
    

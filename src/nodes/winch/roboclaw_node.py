#!/usr/bin/env python

from math import pi, cos, sin

import diagnostic_msgs
import roboclaw_driver as roboclaw
from std_msgs.msg import Int16
import rospy


# QPPS is the speed of the encoder when the motor is at 100% power
# adresse 65 -> SpeedAccelDeccelPositionM1(adress, accel, speed, deccel, position, buffer)
# For an accel=speed in QPPS, the speed is reached in 1 sec
# For an accel=0,5speed in QPPS, the speed is reached in 2sec
# Our motor is 1150 RPM, there are 145.6 tick of encoder per revolution.
# We have an engrenage 24:1 ratio, so 1 output revolution = 24*145.6 = 3494.4ticks
# 1mm = 37.715 ticks (386 mm correspond to 14558 ticks)
# 2775 tick/sec max speed (blocked at 1500) -> 9,705 mm/sec max speed
# max height robot 1076 mm    min height 690 mm -> 390 mm delta
# max 14558 ticks to reach 1076 mm height
# 0 tick -> 690 mm height
# 14558 ticks --> 1076 mm height


class Node:
    def __init__(self):

      
        #0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
        #0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        rospy.init_node("winch_node",log_level=rospy.DEBUG)
        self.pub = rospy.Publisher('winch_Height', Int16, queue_size=50)
        rospy.Subscriber("cmd_winch", Int16, self.run)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        dev_name = rospy.get_param("~dev", "/dev/winch")
        baud_rate = int(rospy.get_param("~baud", "115200"))

        self.address = 0x80
        self.minHeight = 690 #mm
        self.maxHeight = 1076 #mm
        
        try:
            roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")
      
        try:
            version = roboclaw.ReadVersion(self.address)
        except Exception as e:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(e)
            pass

        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.logdebug(repr(version[1]))


        roboclaw.SpeedM1(self.address, 0)
        roboclaw.ResetEncoders(self.address)
       
        self.MAX_SPEED = float(rospy.get_param("~max_speed", "2.0"))
        self.TICKS_PER_METER = float(rospy.get_param("~tick_per_meter", "37715"))
        
        self.last_set_speed_time = rospy.get_rostime()

        rospy.sleep(1)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address %d", self.address)
        rospy.logdebug("max_speed %f", self.MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
      
    

    def displayspeed():
        enc = roboclaw.ReadEncM1(self.address)
        speed = roboclaw.ReadSpeedM1(self.address)
    
        if(enc[0]==1):
            rospy.logdebug("Encoder:",enc[1])
        else:
            rospy.logerr("encoder failed")

        if(speed[0]):
            rospy.logdebug("speed mm/s: %f ",round((speed[1]*9.705)/1500,2))
            rospy.logdebug ("speed: %d",speed[1])
        else:
            rospy.logerr("speed failed")

    

    def run(self,height):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(10)
        while not rospy.is_shutdown():

            if (height < self.minHeight or  height > self.maxHeight):
                rospy.logwarn("Wrong distance")
            else:
                height-= minHeight
                goal = int(37.715*height)
                rospy.logdebug("Position goal: ",goal)
                roboclaw.SpeedAccelDeccelPositionM1(self.address,800,1500,800,goal,0)

            while(roboclaw.ReadEncM1(self.address)[1]!= goal):
                self.displayspeed()
            rospy.loginfo("End")

            """
            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 1:
                rospy.loginfo("Did not get command for 1 second, stopping")
                try:
                    roboclaw.ForwardM1(self.address, 1500)
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)

            status1, enc1, crc1 = None, None, None
           
            try:
                status1, enc1, crc1 = roboclaw.ReadEncM1(self.address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
                rospy.logdebug(e)

            if ('enc1' in vars()):
                rospy.logdebug(" Encoders %d", enc1)
                self.updater.update()
            r_time.sleep()
            """

    

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            roboclaw.SpeedM1(self.address, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.SpeedM1(self.address, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)
    



if __name__ == "__main__":
    try:
        node = Node()
        #node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
    rospy.spin()

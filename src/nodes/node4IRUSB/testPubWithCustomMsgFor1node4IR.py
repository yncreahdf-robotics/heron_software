#!/usr/bin/env python3


from irsensors import IRSensorSet

from rospy import Publisher, init_node, Rate, is_shutdown, get_time, loginfo, ROSInterruptException
from std_msgs.msg import Float64MultiArray


sensor = IRSensorSet("/dev/ttyACM0", [3, 1, 2, 0])
try:
	pub = Publisher("irsensors", Float64MultiArray, queue_size=10)
	init_node("Heron", anonymous=True)
	rate = Rate(10)

	sensor.start()
	array = Float64MultiArray()
	while not is_shutdown():
		array.data = []
		for irsensor in getattr(sensor, "_sensors"):
			array.data.append(irsensor["distance"])	# In milimeters
		loginfo(array)
		pub.publish(array)
		rate.sleep()

except KeyboardInterrupt:
	pass
except ROSInterruptException:
	pass

finally:
	sensor.stop()
	del sensor


#!/usr/bin/env python3
# coding: utf-8


from os import chdir

from heron import Battery
from rospy import get_param, has_param


configFile = "battery.json"
paramStr = "/heron01/battery_sensor/portUSB"
defaultPort = "/dev/battery"


if __name__ == "__main__":
	try:
		chdir("/".join(__file__.split("/")[:-1]))
		battery = Battery(
			configFile,
			get_param(paramStr) if has_param(paramStr) else defaultPort
		)
		battery.launch()
	except KeyboardInterrupt:
		pass
	finally:
		pass

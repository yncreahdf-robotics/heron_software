#!/usr/bin/env python3


from os import chdir
from json import load

from serial import Serial
from rospy import Publisher, init_node, is_shutdown, ROSInterruptException, get_param, has_param
from heron.msg import Battery as BatteryMsg


paramStr = "/heron01/battery_sensor/portUSB"


tests = [
	"00023e05870a0000052400001194000000",
	"ff64330530750000b1010000aef4030098",
	"ff64330530750000ad0100005ced03003b",
	"fe64330530750000c40100005ced030052",
	"ff6433052f750000b50100005ced030042",
	"fe643305307500009401000014f70300e4",
	"fe643305307500009c01000014f70300ec"
]


class Battery:

	NB_BYTES = 17

	def __init__(self, config_file_path, port):
		chdir("/".join(__file__.split("/")[:-1]))
		self.USBcomm = Serial(port, 19200, timeout=1)
		self.USBcomm.close()
		with open(config_file_path, "r") as file:
			self.dataSet = load(file)

	def __getitem__(self, item_name):
		for data in self.dataSet:
			if item_name == data["name"]:
				return data

	def process(self, USBdata):
		if len(USBdata) != Battery.NB_BYTES * 2:
			return False

		checkSum = sum([int(USBdata[2*i:2*i+2], base=16) for i in range(Battery.NB_BYTES)][1:-1])%256

		index = 0
		for data in self.dataSet:
			if "nb_byte" in data.keys():
				# Get all byte for the data
				data["value"] = USBdata[index:index+data["nb_byte"]*2]
				index += data["nb_byte"]*2

				# Reorder bytes (do not have an effect if there is 1 byte in the data)
				temp = data["value"][-2:]
				for i in range(1, data["nb_byte"]):
					temp += data["value"][-(i+1)*2:-i*2]
				data["value"] = temp

				# Get the base 10 value
				data["value"] = int(data["value"], base=16)
				if "coeff" in data.keys():
					data["value"] /= data["coeff"]

			elif data["name"] == "Power":
				data["value"] = round(self["Voltage"]["value"] * self["Current"]["value"], 2)

		return self.check(checkSum)

	def check(self, check_sum):
		if not self["First byte"]["value"] in range(256):
			return False
		if not self["Percentage"]["value"] in range(101):
			return False
		if not self["Voltage"]["value"]*self["Voltage"]["coeff"] in range(50001):
			return False
		if not self["Capacity"]["value"]*self["Capacity"]["coeff"] in range(100, 5000000):
			return False
		if not self["Current"]["value"]*self["Current"]["coeff"] in range(750001):
			return False
		if not self["Remaining Time"]["value"] in range(360000):
			return False
		if self["Check sum"]["value"] != check_sum:
			return False
		return True

	def initROS(self):
		init_node("battery_sensor")
		self.publisher = Publisher("battery", BatteryMsg, queue_size=5)
		self.msg = BatteryMsg()
		for data in self.dataSet:
			if "topic_name" and "unit" in data.keys():
				getattr(self.msg, data["topic_name"]).unit = data["unit"]

	def publish(self):
		for data in self.dataSet:
			if "topic_name" in data.keys():
				getattr(self.msg, data["topic_name"]).value = data["value"]
		self.publisher.publish(self.msg)

	def launch(self):
		try:
			self.initROS()
			self.USBcomm.open()
			while not is_shutdown():
				if self.process(self.USBcomm.readline().hex()):
					self.publish()
				print(self)
		except ROSInterruptException:
			pass

	def __str__(self):
		string = "{\n"
		for data in self.dataSet:
			string +="\t" + str(data["name"])
			if "value" in data.keys():
				string += ": " + str(data["value"])
			if "unit" in data.keys():
				string += " " + str(data["unit"])
			string += ",\n"
		string = string[:-2] + "\n}"
		return string

	def __del__(self):
		self.USBcomm.close()


class BatteryTest():
	def __init__(self, port="/dev/ttyUSB0"):
		self.USBcomm = Serial(port, 19200, timeout=1)
		while True:
			print(self.USBcomm.readline().hex())


if __name__ == "__main__":
	battery = Battery("config/battery.json", get_param(paramStr) if has_param(paramStr) else "/dev/ttyUSB0")
	battery.launch()

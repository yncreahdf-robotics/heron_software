#!/usr/bin/env python3


from json import load
from copy import deepcopy

from serial import Serial
from rospy import Publisher, init_node, is_shutdown, ROSInterruptException
from heron.msg import Battery as BatteryMsg


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

	NAME = "Battery"
	NB_BYTES = 17

	def __init__(self, config_file_path, port="/dev/ttyUSB0"):
		object.__setattr__(self, "USBcomm", Serial(port, 19200, timeout=1))
		getattr(self, "USBcomm").close()
		with open(config_file_path, "r") as file:
			object.__setattr__(self, "dataSet", load(file))

	def __setattr__(self, attr_name, attr_value):
		raise TypeError("'" + Battery.NAME + "' object does not support attribute assignment " + \
		"(attributeName > " + attr_name + ", attributeValue > " + str(attr_value))
	def __delattr__(self, attr_name):
		raise TypeError("'" + Battery.NAME + "' object does not support attribute deletion " + \
		"(attributeName > " + attr_name + ")")
	def __getitem__(self, item_name):
		if item_name == "list":
			return deepcopy(self.dataSet)
		for data in self.dataSet:
			if item_name == data["name"]:
				tmp = {}
				for key, value in data.items():
					tmp[key] = value
				return tmp
		raise KeyError("'" + Battery.NAME + "' object has no item '" + item_name + "'")
	def __setitem__(self, item_name, item_value):
		 raise TypeError("'" + Battery.NAME + "' object does not support item assignment " + \
		"(attributeName > " + item_name + ", attributeValue > " + str(item_value) + ")")
	def __delitem__(self, item_name):
		raise TypeError("'" + Battery.NAME + "' object does not support item deletion " + \
		"(attributeName > " + item_name + ")")

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
		if not self["Voltage"]["value"]*100 in range(50001):
			return False
		if not self["Capacity"]["value"]*1000 in range(100, 5000000):
			return False
		if not self["Current"]["value"]*1000 in range(750001):
			return False
		if not self["Remaining Time"]["value"] in range(360000):
			return False
		if self["Check sum"]["value"] != check_sum:
			return False
		return True

	def initROS(self):
		init_node("Heron", anonymous=True)
		object.__setattr__(self, "publisher", Publisher(Battery.NAME.lower(), BatteryMsg, queue_size=5))
		object.__setattr__(self, "msg", BatteryMsg())
		for data in self.dataSet:
			if "topic_value" and "unit" in data.keys():
				getattr(self.msg, data["topic_value"]).unit = data["unit"]

	def publish(self):
		for data in self.dataSet:
			if "topic_value" in data.keys():
				getattr(self.msg, data["topic_value"]).value = data["value"]
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
	battery = Battery("config/battery.json")
	battery.launch()

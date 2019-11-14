#!/usr/bin/env python3
# coding: utf-8


from typing import Union


def getNbByte(number: Union[int, str, bytes]) -> int:
	if type(number) == int:					# If number is an int-object, we use the format function to get the associated str-object with hexadecimal digits.
		number = format(number, "x")
	if type(number) == str:					# If number is a str-object, we use the fromhex method from bytes class to get the bytes-object coded in hexadecimal.
		if(len(number) % 2 == 1):				# If the number of digits is odd, we must add ourself a 0 digit at the beginning to have an even number of digit for the bytes.fromhex() method.
			number = "0" + number
		number = bytes.fromhex(number)
	if type(number) != bytes:				# We check that number is a bytes-object.
		raise TypeError("number must be an 'int', a 'bytes' encode with hexadecimal value or a 'str' with hexadecimal digits. number: " + str(number))
	return(len(number))						# The function len() gives us the number of bytes.


class Converter:
	# ID MSG
	STANDARD, LARGE = (0, 1)	# ID MSG type
	ID_MSG_TYPE_SIZE = {		# ID MSG size in byte
		STANDARD: 2,
		LARGE: 4
	}
	# TYPE FRAME
	DATA, REMOTE = (0, 1)
	TYPE_FRAME = (DATA, REMOTE)

	FIRST_BYTE = 170	# Constant use for the USB protocol with the converter USB/CAN.
	LAST_BYTE = 85		# Constant use for the USB protocol with the converter USB/CAN.

	def __init__(self, idMsgType: int = STANDARD):
		if idMsgType not in Converter.ID_MSG_TYPE_SIZE.keys():	# We check that the idMsgType is a suitable value.
			raise ValueError("Not a valid idMsgSize: " + str(idMsgType))
		self.idMsgType = idMsgType

	def createConfigByte(self, idMsg: Union[int, str], payload: Union[int, str], typeFrame: int = DATA) -> int:
		idMsgNbByte = getNbByte(idMsg)					# We check that the number of bytes for the msg ID is not too large.
		if idMsgNbByte > Converter.ID_MSG_TYPE_SIZE[self.idMsgType]:
			raise ValueError("idMsg too large: " + str(idMsg))
		payloadNbByte = getNbByte(payload)	# We check that payload is not too to large.
		if payloadNbByte > 0b1111:
			raise ValueError("payload to large. It must contain at the most " + str(0b1111) + " bytes. payload: " + str(payload) + ", nb bytes: " + str(payloadNbByte))
		if typeFrame not in Converter.TYPE_FRAME:		# We check that typeFrame is either DATA or REMOTE.
			raise ValueError("Not a valid type frame: " + str(typeFrame))

		configByte  = 0b11000000												# We set the 2 first bits to 1 for the converter USB/CAN.
		configByte += 0b00100000 if self.idMsgType == Converter.LARGE else 0	# If necessary, we change the bit associated with the number of byte in the msg id.
		configByte += 0b00010000 if typeFrame == Converter.REMOTE else 0		# If necessary, we change the bit associated with the msg type (REMOTE ou DATA).
		configByte += getNbByte(payload)										# We add the number of bytes for the payload
		return configByte

	def createMsgID(serviceID: int, nodeID: int) -> int:
		if serviceID > 0b11111:		# We check that serviceID is not too large.
			raise ValueError("serviceID is too large: " + str(serviceID))
		if nodeID > 0b111111:		# We check that nodeID is not too large.
			raise ValueError("nodeId is too large: " + str(nodeID))
		# The msg ID contains the nodeID on his 6 less signiicant bits, and the service ID on his next 5.
		serviceID = format(serviceID, "010b")
		msgID = serviceID + format(nodeID, "06b")
		msgID = int(msgID, base=2)
		return msgID
	createMsgID = staticmethod(createMsgID)

	def reverseMsgID(msgID: int) -> int:
		msgID = format(msgID, "x")
		if len(msgID) % 2 == 1:
			msgID = "0" + msgID
		reverseMsgID = msgID[-2:]
		for i in range(1, len(msgID)):
			reverseMsgID += msgID[-(i+1)*2:-i*2]
		reverseMsgID = int(reverseMsgID, base=16)
		return reverseMsgID
	reverseMsgID = staticmethod(reverseMsgID)


def printByte(nombre: int) -> None:
	nombre = format(nombre, "08b")			# We create the associated str-object with the binary number
	for i in range(len(nombre)):			# We print each digit one by one
		print(nombre[i], end="")				# We have to precise at the print function to not go on the next line with the end parameter
		if i == 3:								# If the the last digit printed is the third, wu put an espace
			print(end=" ")
	print()


if __name__ == "__main__":
	try:
		print(format(Converter.reverseMsgID(int("3202ab", base=16)), "x"))
	except KeyboardInterrupt:
		pass
	finally:
		pass

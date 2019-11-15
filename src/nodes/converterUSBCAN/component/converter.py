#!/usr/bin/env python3
# coding: utf-8


from typing import Union
try:
	from useful import getNbByte
except ImportError:
	from component.useful import getNbByte


class Converter:
	# ID MSG
	STANDARD, EXTENDED = (0, 1)	# ID MSG type
	ID_MSG_TYPE_SIZE = {		# ID MSG size in byte
		STANDARD: 2,
		EXTENDED: 4
	}
	# TYPE FRAME
	DATA, REMOTE = (0, 1)
	TYPE_FRAME = (DATA, REMOTE)

	FIRST_BYTE = b'\xaa'	# Constant use for the USB protocol with the converter USB/CAN.
	LAST_BYTE = b'\x55'		# Constant use for the USB protocol with the converter USB/CAN.

	def __init__(self, idMsgType: int = STANDARD):
		if idMsgType not in Converter.ID_MSG_TYPE_SIZE.keys():	# We check that the idMsgType is a suitable value.
			raise ValueError("Not a valid idMsgSize: " + str(idMsgType))
		self.idMsgType = idMsgType

	def createConfigByte(self, idMsg: Union[int, str], payload: Union[int, str], typeFrame: int = DATA) -> bytes:
		idMsgNbByte = getNbByte(idMsg)					# We check that the number of bytes for the msg ID is not too large.
		if idMsgNbByte > Converter.ID_MSG_TYPE_SIZE[self.idMsgType]:
			raise ValueError("idMsg too large: " + str(idMsg))
		payloadNbByte = getNbByte(payload)	# We check that payload is not too to large.
		if payloadNbByte > 0b1111:
			raise ValueError("payload to large. It must contain at the most " + str(0b1111) + " bytes. payload: " + str(payload) + ", nb bytes: " + str(payloadNbByte))
		if typeFrame not in Converter.TYPE_FRAME:		# We check that typeFrame is either DATA or REMOTE.
			raise ValueError("Not a valid type frame: " + str(typeFrame))

		configByte  = 0b11000000												# We set the 2 first bits to 1 for the converter USB/CAN.
		configByte += 0b00100000 if self.idMsgType == Converter.EXTENDED else 0	# If necessary, we change the bit associated with the number of byte in the msg id.
		configByte += 0b00010000 if typeFrame == Converter.REMOTE else 0		# If necessary, we change the bit associated with the msg type (REMOTE ou DATA).
		configByte += getNbByte(payload)										# We add the number of bytes for the payload
		return bytes.fromhex(format(configByte, "02x"))

	def createMsgID(serviceID: int, nodeID: int) -> int:
		if serviceID > 0b11111:		# We check that serviceID is not too large.
			raise ValueError("serviceID is too large: " + str(serviceID))
		if nodeID > 0b111111:		# We check that nodeID is not too large.
			raise ValueError("nodeId is too large: " + str(nodeID))
		# The msg ID contains the nodeID on his 6 less signiicant bits, and the service ID on his next 5.
		msgID = (serviceID << 6) + nodeID	# In binary, we add the nodeID in the less significant bit
		return msgID
	createMsgID = staticmethod(createMsgID)

	def reverseMsgID(msgID: int) -> bytes:
		reverseMsgID = ""
		for byte in bytes.fromhex(format(msgID, "04x")):	# for each byte in msgID (it is read from left to right)
			reverseMsgID = format(byte,'02x') + reverseMsgID	# We concatenate the byte read in the most significant bit
		return bytes.fromhex(reverseMsgID)
	reverseMsgID = staticmethod(reverseMsgID)

	def createPayload(data: str) -> bytes:
		if len(data) > 0b1111*2:
			raise ValueError("Too many hexadecimal digits.")

		if len(data) % 2 == 1:
			data = "0" + data
		return bytes.fromhex(data)
	createPayload = staticmethod(createPayload)


if __name__ == "__main__":
	try:
		print(Converter.createPayload("ffaa"))
	except KeyboardInterrupt:
		pass
	finally:
		pass

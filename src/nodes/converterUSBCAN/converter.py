#!/usr/bin/env python3
# coding: utf-8


from typing import Union


def getNbByte(number: Union[int, str, bytes]) -> int:
	if type(number) == int:					# If number is an int-object, we use the format function to get the associated str-object with hexadecimal digits.
		number = format(number, "x")
	if type(number) == str:					# If number is a str-object, we use the fromhex method from bytes class to get the bytes-object coded in hexadecimal.
		if(len(number)%2 == 1):				# If the number of digits is odd, we must add ourself a 0 digit at the beginning to have an even number of digit for the bytes.fromhex() method.
			number = "0"+number
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
		idMsgNbByte = getNbByte(idMsg)				# We check that the number of bytes for the msg ID is not too large.
		if idMsgNbByte > Converter.ID_MSG_TYPE_SIZE[self.idMsgType]:
			raise ValueError("idMsg too large: " + str(idMsg))

		configByte  = 0b11000000												# We set the 2 first bits to 1 for the converter USB/CAN.
		configByte += 0b00100000 if self.idMsgType == Converter.LARGE else 0	# If necessary, we change the bit associated with the number of byte in the msg id.
		configByte += 0b00010000 if typeFrame == Converter.REMOTE else 0		# If necessary, we change the bit associated with the msg type (REMOTE ou DATA).
		configByte += getNbByte(payload)										# We add the number of bytes for the payload
		return configByte


def printByte(nombre: int) -> None:
	nombre = format(nombre, "08b")			# We create the associated str-object with the binary number
	for i in range(len(nombre)):			# We print each digit one by one
		print(nombre[i], end="")				# We have to precise at the print function to not go on the next line with the end parameter
		if i == 3:								# If the the last digit printed is the third, wu put an espace
			print(end=" ")
	print()


if __name__ == "__main__":
	try:
		converter = Converter()
		printByte(converter.createConfigByte(0xffff, "", 0))
		printByte(converter.createConfigByte(0xffff, "", 1))
		printByte(converter.createConfigByte(0xffff, "2", 0))
		printByte(converter.createConfigByte(0xffff, "a2", 0))
		printByte(converter.createConfigByte(0xffff, "fa2", 0))
		converter = Converter(Converter.LARGE)
		printByte(converter.createConfigByte(0x10000, "", 0))
		printByte(converter.createConfigByte(0x10000, "", 1))
	except KeyboardInterrupt:
		pass
	finally:
		pass

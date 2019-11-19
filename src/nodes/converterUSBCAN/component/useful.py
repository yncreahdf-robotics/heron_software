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

def printByte(nombre: int) -> None:
	nombre = format(nombre, "08b")			# We create the associated str-object with the binary number
	for i in range(len(nombre)):			# We print each digit one by one
		print(nombre[i], end="")				# We have to precise at the print function to not go on the next line with the end parameter
		if i == 3:								# If the the last digit printed is the third, wu put an espace
			print(end=" ")
	print()


if __name__ == "__main__":
    try:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        pass

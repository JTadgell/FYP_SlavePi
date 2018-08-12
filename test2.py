import numpy as np
import serial

def func2():
	ser = serial.Serial(
	    port='/dev/ttyACM0',
	    baudrate = 57600,
	    parity=serial.PARITY_NONE,
	    stopbits=serial.STOPBITS_ONE,
	    bytesize=serial.EIGHTBITS,
	    timeout=1)

	print("func2: "+str(ser.name))
	return ser
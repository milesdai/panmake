import serial
import time 

PORT_NAME = 'COM3'
ser = serial.Serial(PORT_NAME, 9600, timeout=0)

message = b'1'
print('Writing: ' + str(message))
ser.write(message)

import serial
import time 

PORT_NAME = '/dev/cu.usbmodem14401'
ser = serial.Serial(PORT_NAME, 9600, timeout=0)

message = b'$1111111111111!'
print('Writing: ' + str(message))

ser.write(message)
while True:
    ser.flush
    print(ser.readline())
    time.sleep(0.5)
    message = b'$1010101111100000!'
    print('Writing: ' + str(message))

    ser.write(message)
    #ser.write(1)

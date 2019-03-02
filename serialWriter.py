import serial
import time 

PORT_NAME = '/dev/cu.usbmodem14401'
ser = serial.Serial(PORT_NAME, 9600, timeout=0)

def sendMessage(message):
    '''message should be a byte string. Start and end characters appended automatically'''
    message = b'$' + message + b'!'
    print('Writing to Arduino: ' + str(message))
    ser.write(message)
    ser.flush()

sendMessage('50,20,100,1')
sendMessage('60,70,150,0')

import serial
import time 

PORT_NAME = '/dev/cu.usbmodem14401'
PORT_NAME = 'COM10'
ser = serial.Serial(PORT_NAME, 115200, timeout=0)

def sendMessage(message):
    '''message should be a byte string. Start and end characters appended automatically'''
    # message = b'$' + message.encode('utf-8') + b'!'
    message = message.encode('utf-8') + b'!'
    print('Writing to Arduino: ' + str(message))
    ser.write(message)
    ser.flush()

def readMessage():
    print("Reading from Arduino: " + str(ser.readline()))

while True:
    sendMessage('50,20,100,1')
    readMessage()
    time.sleep(2)
    sendMessage('60,70,150,0')
    readMessage()
    time.sleep(2)

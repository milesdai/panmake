import serial
import time 

PORT_NAME = '/dev/cu.usbmodem14401'
PORT_NAME = 'COM10'
ser = serial.Serial(PORT_NAME, 115200, timeout=0)
time.sleep(3)
ser.flush()

def sendMessage(message):
    '''message should be a byte string. Start and end characters appended automatically'''
    while True:
        rx = ser.readline()
        if len(rx) > 0:
            print(rx)
        time.sleep(0.01)
        if len(rx) > 0 and rx == b'>\r\n':
            message = message + b'!'
            print('Writing to Arduino: ' + str(message))
            ser.write(message)
            ser.flush()
            message = b''
            break

def readMessage():
    print("Reading from Arduino: " + str(ser.readline()))

while True:
    sendMessage(b'50,20,100,1')
    readMessage()
    time.sleep(3)
    sendMessage(b'60,70,150,0')
    readMessage()
    time.sleep(3)

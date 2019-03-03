import serial
import time 

# PORT_NAME = '/dev/cu.usbmodem14401'
PORT_NAME = 'COM10'
X_LOW_LIMIT = 0
Y_LOW_LIMIT = 0
X_HIGH_LIMIT = 3000
Y_HIGH_LIMIT = 3000
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


# while True:
#     sendMessage(b'50,20,100,1')
#     readMessage()
#     time.sleep(3)
#     sendMessage(b'60,70,150,0')
#     readMessage()
#     time.sleep(3)
def getCommand(filename):
    f = open(filename, 'r')
    x_val = 0
    y_val = 0
    extruder = -1
    for line in f:
        line_split = line.split()
        
        extruder = chr(ord(line_split[0])) #Converts single character string into character
        if extruder == 'w':
            time.sleep(25)
        else:
            x_val = int(line_split[1])
            y_val = int(line_split[2])
            if extruder == -1:
                raise Exception
            if x_val < X_LOW_LIMIT or x_val > X_HIGH_LIMIT:
                raise Exception
            if y_val < Y_LOW_LIMIT or y_val > Y_HIGH_LIMIT:
                raise Exception
            if extruder == 'm':
                e = b'0'
            else:
                e = b'1'
            # str(extruder).encode('utf-8')
            yield b'' + str(int(1.25 * x_val)).encode('utf-8') + b',' + str(int(1.25*y_val)).encode('utf-8') +b',' + b'10,' + e + b'!'
        
            
            
for command in getCommand("trace/square.txt"):
    print("Command is ", command)
    sendMessage(command)
    readMessage()
    time.sleep(1)

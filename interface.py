import serial
import matplotlib.pyplot as plt
from drawnow import *
import atexit

values = []

plt.ion()
cnt=0

serialArduino = serial.Serial('/dev/ttyUSB0', 9600)
# serialArduino.bytesize = 8
serialArduino.parity = serial.PARITY_NONE
serialArduino.stopbits = serial.STOPBITS_ONE
serialArduino.timeout = None


def plotValues():
    plt.title('PWM_MOTOR')
    plt.grid(True)
    plt.ylabel('PWM')
    plt.plot(values, 'rx-', label='values')
    plt.legend(loc='upper right')

def doAtExit():
    serialArduino.close()
    print("Close serial")
    print(str("serialArduino.isOpen() = ") + str(serialArduino.isOpen()))

atexit.register(doAtExit)

print(str("serialArduino.isOpen() = ") + str(serialArduino.isOpen()))

#pre-load dummy data
for i in range(0,200):
    values.append(0)

serialArduino.write(b'0x01')
serialArduino.flushOutput()
while True:
    while (serialArduino.inWaiting()==0):
        pass
    valueRead = serialArduino.read()
    serialArduino.flushInput()


    valueInInt = int.from_bytes(valueRead,byteorder='little')
    if valueInInt <= 255:
        if valueInInt >= 0:
            values.append(valueInInt)
            values.pop(0)
            drawnow(plotValues)

        else:
            print("Invalid! negative number")
    else:
        print("Invalid! too large")

serialArduino.flush()
serialArduino.close()

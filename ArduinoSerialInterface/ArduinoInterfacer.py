import serial
import signal
import sys
from time import sleep

def signal_handler(signal, frame): ##lets you use ctrl-C to terminate
        print('Terminating')
        ser.write("90,90f")
        ser.close()
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

ser = serial.Serial(port = 'COM5',baudrate = 250000)
if ser.isOpen():
    ser.close()
ser.open()


print('Connection Success')

line = []
while True:
        #Figure out how much data there is to be read, and read all of it
        a = ser.inWaiting()
        b = ser.read(a).splitlines()
        ##Sometimes, a partial message is read, so we read the second-to-last data package
        if (len(b)>1):
                print(b[-2])
        ser.write("95,95f")
        sleep(.01) ##We read from the Arduino a little slower than it parses data

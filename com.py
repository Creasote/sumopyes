import serial
import time

arduino=serial.Serial('/dev/ttyS2', 9600)
time.sleep(2)


def write(var):
    arduino.write(bytes(var,'ASCII'))   
    return

def read():
    return arduino.read_all()


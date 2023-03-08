import serial
import time

arduino=serial.Serial('/dev/ttyS2', 9600)
time.sleep(2)


while 1:
    
    userInput=input()
    userInput.strip()
    if (userInput=='w'):
        arduino.write(bytes(userInput,'ASCII'))        
    elif (userInput=='a'):
        arduino.write(b'a')    

    arduino.write(b's')

    reply = arduino.read_all()

    print(reply)


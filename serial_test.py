import serial
import time

arduino=serial.Serial('/dev/ttyS2', 9600)
time.sleep(2)

speed = 4

# while 1:
    
# userInput=input()
# userInput.strip()
# if (userInput=='w'):
#     arduino.write(bytes(userInput,'ASCII'))        
# elif (userInput=='a'):
#     arduino.write(b'a')    

arduino.write(b's')
time.sleep(2)
reply = arduino.read_all()
print(reply)


#arduino.write(bytes(str(speed),'ASCII'))
arduino.write(bytes(chr(speed),'utf-8'))#,'ASCII')
time.sleep(2)
reply = arduino.read_all()
print(reply)
time.sleep(2)

reply = arduino.read_all()
print(reply)



import serial
import time

arduino=serial.Serial('/dev/ttyS2', 9600)
time.sleep(2)

print("Enter   1 to turn ON LED and 0 to turn OFF LED")

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

    # if datafromUser == '1':
    #     arduino.write(b'1')
    #     print("LED   turned ON")
    # elif datafromUser == '0':
    #     arduino.write(b'0')
    #     print("LED turned OFF")

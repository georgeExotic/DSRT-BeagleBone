#!/usr/bin/python3
from base64 import encode
import socket
import json
from time import sleep
import rcpy
import rcpy.motor as motor
import rcpy.adc as adc



IP = "192.168.1.1"
PORT = 3553

robotSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
robotSocket.bind((IP,PORT))
robotSocket.settimeout(0.25)


#dashboard inputs#
leftJoystick = 0
rightJoystick = 0
speedMultiplier = 0.5 
button = False
switch = False

#robot outputs#
robotVoltage = 0


def recieveData():
    global leftJoystick,rightJoystick,speedMultiplier,button,switch

    try:
        inputData,recvAddr = robotSocket.recvfrom(1024)
        inputData = json.loads(inputData)
    except socket.timeout:
        inputData = None

    if inputData is not None:

        if 'leftJoystick' in inputData.keys():
            leftJoystick = inputData['leftJoystick'] 
        
        if 'rightJoystick' in inputData.keys():
            rightJoystick = inputData['rightJoystick'] 

        if 'speed' in inputData.keys():
            speedMultiplier = inputData['speed']

        if 'button' in inputData.keys():
            button = inputData['button']

        if 'lightSwitch' in inputData.keys():
            switch = inputData['lightSwitch']

    # print(('left Motor = {}\tright Motor = {}\t SM = {}\t Btn = {}\t Switch = {}'.format(leftJoystick,rightJoystick,speedMultiplier,button,switch)))


def joystickControl():

    rightMotors = 1
    leftMotors = 2

    rightMotorSpeed = (rightJoystick*speedMultiplier)
    leftMotorSpeed = -1*(leftJoystick*speedMultiplier)

    motor.set(rightMotors,rightMotorSpeed)
    motor.set(leftMotors,leftMotorSpeed)
      

def sendData():
    robotBatery = round(adc.dc_jack.get_voltage(),4)
    
    data = {"robotBatery" : robotBatery}
    data = json.dumps(data)
    robotSocket.sendto(data.encode('utf-8'),(IP,3554))
         
if __name__ == "__main__":

    while True:
        recieveData()
        joystickControl()
        sendData()
        sleep(0.01)
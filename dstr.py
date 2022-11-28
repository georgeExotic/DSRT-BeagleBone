#!/usr/bin/python3
import socket
import threading
import json
from time import sleep
import numpy as np
import DSTRdriverv1
import rcpy
import rcpy.adc as adc



##inverse kinematics## 
# x_dot is the y axis in the joystick 
# theta_dot is the x axis in the joystick

class DSTR:

    def __init__(self):

        #Kinematics#
        self.wheelRadius = 0.04
        self.wheelBase = 0.1
        self.A_matrix = np.array([[1/self.wheelRadius, -self.wheelBase/self.wheelRadius], [1/self.wheelRadius, self.wheelBase/self.wheelRadius]])
        self.max_xd = 0.4
        self.max_td = (self.max_xd/self.wheelBase)

        #robot Data#
        self.batteryPackVoltage = 0

        #UPD communication#
        self.IP = "192.168.2.1"
        self.port = 3553
        
        self.joysticksock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.joysticksock.bind((self.IP, self.port))
        self.joysticksock.settimeout(.25)

        ##joystick Thread##
        self.joystickData = {
                            "x" :   0,
                            "y" :   0,
                            "button" : 1
                            }

        self.robotData =    {
                            "robotVoltage" : 0,
                            "IMU" : 0
                            }
                         

        try:
            rcpy.set_state(rcpy.RUNNING)
        except:
            print("There is an issue with the Python 3 Interface for the Robotics Cape on the Beaglebone Blue")
        
        self.joystickUpdateThread = threading.Thread(target=self._joystickUpdater)
        self.joystickUpdateThread.start()

        self.controlThread = threading.Thread(target=self._controlUpdater)
        self.controlThread.start()

        self.robotDataUpdaterThread = threading.Thread(target=self._robotDataUpdater)
        self.robotDataUpdaterThread.start()

    def _joystickUpdater(self):
        while True:
            self._getJoystickData()

    def _getJoystickData(self):
        try:
            joystickData,recvAddr = self.joysticksock.recvfrom(1024)
            # if 'x' and 'y' in joystickData:

            self.joystickData = json.loads(joystickData)
            print(joystickData)

        except socket.timeout:
            self.joystickData = None

    def _controlUpdater(self):
         while rcpy.get_state() != rcpy.EXITING:     # exit loop if rcpy not ready
            if rcpy.get_state() == rcpy.RUNNING:    # execute loop when rcpy is ready
                if self.joystickData != None:
                    try:
                        userInputTarget = self.getJoystickData()
                        wheelSpeedTarget = self._getWheelSpeed(userInputTarget)
                        DSTRdriverv1.driveOpenLoop(wheelSpeedTarget, SM=0.8)
                    except: 
                        pass

    def _getWheelSpeed(self,userInputTarget):
        try:
            robotTarget = self._mapSpeeds(np.array([userInputTarget.get("x")*-1,userInputTarget.get("y")*-1]))
            wheelSpeedTarget = self._calculateWheelSpeed(robotTarget)
            return wheelSpeedTarget
        except:
            pass

    def _mapSpeeds(self,original_B_matrix):
        B_matrix = np.zeros(2)
        B_matrix[0] = self.max_xd * original_B_matrix[0]
        B_matrix[1] = self.max_td * original_B_matrix[1]
        return B_matrix

    def _calculateWheelSpeed(self,B_matrix):
        C_matrix = np.matmul(self.A_matrix,B_matrix)
        C_matrix = np.round(C_matrix,decimals=3)
        return C_matrix

    def _robotDataUpdater(self): 
        while True:
            self._getRobotData()

    def _getRobotData(self):
        try:
            self.batteryPackVoltage = adc.dc_jack.get_voltage()
            self.robotData["robotVoltage"] = round(self.batteryPackVoltage,4)
        except:
            self.batteryPackVoltage = None
    
    def getRobotData(self):
        return self.robotData

    def getJoystickData(self):
        return self.joystickData

    def sendRobotData(self):
        with open('/tmp/robotData.json','w') as write_file:
                json.dump(self.getRobotData(),write_file,indent=4)


         
if __name__ == "__main__":

    robot = DSTR()
    while True:
        robot.sendRobotData()
        sleep(1)
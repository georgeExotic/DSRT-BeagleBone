#!/usr/bin/python3
import socket
import threading
import json
from time import sleep
import numpy as np
# import L1_motor as motor
# import L2_inverse_kinematics
import L2_speed_control as sc

class SCUTTLE:

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
        self.IP = "127.0.0.1"
        self.port = 3553
        self.dashBoardDatasock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dashBoardDatasock.bind((self.IP, self.port))
        self.dashBoardDatasock.settimeout(.25)

        ##joystick Thread##
        self.dashBoardData = None
                         
        ##Start Thread##
        self.dashBoardDataUpdateThread = threading.Thread(target=self._dashBoardDataUpdater)
        self.dashBoardDataUpdateThread.start()

        self.controlThread = threading.Thread(target=self.__controlLoopUpdater)
        self.controlThread.start()

    def _dashBoardDataUpdater(self):
        while True:
            self._dashBoardDataData()

    def __controlLoopUpdater(self):
        while True:
            self._controlUpdater()

    def _dashBoardDataData(self):
        try:
            dashBoardData,recvAddr = self.dashBoardDatasock.recvfrom(1024)
            self.dashBoardData = json.loads(dashBoardData)

        except socket.timeout:
            self.dashBoardData = None

    def _controlUpdater(self):
        if self.dashBoardData != None:
            try:
                userInputTarget = self.dashBoardData['one_joystick']
                wheelSpeedTarget = self._getWheelSpeed(userInputTarget)
                sc.driveOpenLoop(wheelSpeedTarget)
            except: 
                pass

    def _getWheelSpeed(self,userInputTarget):
        try:
            robotTarget = self._mapSpeeds(np.array([userInputTarget['y'],-1*userInputTarget['x']]))
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


    def getdashBoardData(self):
        return self.dashBoardData

         
if __name__ == "__main__":

    robot = SCUTTLE()
    while True:
        if robot.dashBoardData != None:
            # print(robot.dashBoardData['one_joystick']['x'])
            sleep(0.2)
'''
/* =======================================================================
Description:

    This file contains a guard robot class that has a method to calculate
    its virtual position.
   ========================================================================*/
'''

import final_config as gVars
import final_draw as draw
from HamsterAPI.comm_ble import RobotComm
import math
import numpy as numpy
import threading
import time 
import Tkinter as tk 

#define common angles
pi4 = 3.14159/2
pi2 = 3.14159/2
p3i4 = (3*3.14159)/4
pi = 3.14159
p5i4 = (5*3.14159)/4
p3i2 = (3*3.14159)/2
p7i4 = (7*3.14159)/4
p2i = 2 * 3.14159

class Guard:
    def __init__(self, comm, rCanvas):
        self.gMaxRobotNum = gVars.gMaxRobotNum
        self.gRobotList = gVars.grobotList
        self.robot = None  #second robot connected will be guard
        self.m = gVars.m
        self.vrobot = draw.virtual_robot()
        self.vrobot.time = time.time()
 
    #Thread to update virtual robot based on a model of my robot
    def update_virtual_robot(self):
  
        #these values are for a charged robot 031
        #the model is based on the model provided, and if the battery is charged, meets the reqs
        noise_prox = 25 # noisy level for proximity
        noise_floor = 20 #floor ambient color - if floor is darker, set higher noise
        p_factor = 1.4 #proximity conversion - assuming linear
        d_factor = 2.3 #travel distance conversion (large d_factor makes vrobot slower)
        a_factor = 5 #rotation conversion, assuming linear (large a_factor makes vrobot slower)
        wheel_balance = -6 #value for 031. -128(L) ~ 127(R)(0: off), my hamster swerves right

        #wait until robot is connected
        while  len(self.gRobotList) < 2:
            time.sleep(0.1)
        
        while not gVars.gQuit:
            if self.gRobotList[1] is not None:
                #set prisoner robot
                self.robot = gVars.grobotList[1]   #first robot connected will be prisoner
                robot = self.robot
                
                #set initial position
                self.vrobot.x = 0
                self.vrobot.y =  140
                self.vrobot.a  = pi
            
                #set wheel balance
                robot.set_wheel_balance(wheel_balance)
                                
                #update robot position in the GUI                        
                t = time.time()
                del_t = t - self.vrobot.t
                self.vrobot.t = t # update the tick
                if self.vrobot.sl == self.vrobot.sr:
                    self.vrobot.x = self.vrobot.x + self.vrobot.sl * del_t * math.sin(self.vrobot.a) * d_factor
                    self.vrobot.y = self.vrobot.y + self.vrobot.sl * del_t * math.cos(self.vrobot.a) * d_factor
                if self.vrobot.sl == -self.vrobot.sr:
                    self.vrobot.a = self.vrobot.a + (self.vrobot.sl * del_t)/a_factor
                    
                #update sensors
                prox_l = robot.get_proximity(0)
                prox_r = robot.get_proximity(1)
                
                #draw prox lines
                if (prox_l > noise_prox):
                    self.vrobot.dist_l = (100 - prox_l)*p_factor
                else:
                    self.vrobot.dist_l = False
                if (prox_r > noise_prox):
                    self.vrobot.dist_r = (100 - prox_r)*p_factor
                else:
                    self.vrobot.dist_r = False
                
                #draw floor dots
                floor_l = robot.get_floor(0)
                floor_r = robot.get_floor(1)
                if (floor_l < noise_floor):
                    self.vrobot.floor_l = floor_l
                else:
                    self.vrobot.floor_l = False
                if (floor_r < noise_floor):
                    self.vrobot.floor_r = floor_r
                else:
                    self.vrobot.floor_r = False
            time.sleep(0.001)
                    


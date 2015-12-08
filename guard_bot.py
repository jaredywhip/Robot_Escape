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
import Queue

#define common angles
pi4 = 3.14159/2
pi2 = 3.14159/2
p3i4 = (3*3.14159)/4
pi = 3.14159
p5i4 = (5*3.14159)/4
p3i2 = (3*3.14159)/2
p7i4 = (7*3.14159)/4
p2i = 2 * 3.14159

#set endpoint locations
rbox_x = 260
lbox_x = -60

class Guard:
    def __init__(self, comm, rCanvas):
        self.gMaxRobotNum = gVars.gMaxRobotNum
        self.gRobotList = gVars.grobotList
        self.robot = None  #second robot connected will be guard
        self.m = gVars.m
        self.vrobot = draw.virtual_robot()
        self.vrobot.time = time.time()

    #helper functions for the robot 
    def set_linetracer_mode_speed(self, mode, speed):
        is_set = False
        while (not is_set):
            if gVars.grobotList[1]:
                self.robot = gVars.grobotList[1]
                robot = self.robot
                robot.set_line_tracer_mode_speed(mode, speed)
                is_set = True 

    def get_prox(self, side):
        prox = None
        while(prox is None):
            if gVars.grobotList[1] is not None:
                self.robot = gVars.grobotList[1]
                if (side == 0):
                    prox = self.robot.get_proximity(0)
                else:
                    prox = self.robot.get_proximity(1)
        return prox

    def get_floor(self, side):
        floor = None
        while(floor is None):
             if gVars.grobotList[1]:
                self.robot = gVars.grobotList[1]
                if (side == 0):
                    floor = self.robot.get_floor(0)
                else:
                    floor = self.robot.get_floor(1)
        return floor

    # rotates left and right until proximities are even and above 25
    def rotate_till_even_proximities(self):
        if self.gRobotList[1] is not None:
            robot = self.gRobotList[1]
            lprox = robot.get_proximity(0)
            rprox = robot.get_proximity(1)
            while(abs(lprox - rprox) > 2):
                if (rprox < lprox or rprox < 25):
                    self.move_left_slow()
                    time.sleep(.05)
                elif (rprox > lprox or lprox < 25):
                    self.move_right_slow()
                    time.sleep(.05)
                lprox = robot.get_proximity(0)
                rprox = robot.get_proximity(1)
            self.stop_move()
            time.sleep(1)

    def localize(self):
        if (self.vrobot.a % (2*pi) > 0 and self.vrobot.a % (2*pi)  < pi):  # localize to right end box
            self.vrobot.x = rbox_x - 60
        else:   # localize to left slider box
            self.vrobot.x = lbox_x + 60

    def alert_found_pris(self):
         if self.gRobotList:
            robot = self.gRobotList[1]
            robot.set_led(0,2)
            robot.set_led(1,2)
            robot.set_musical_note(90)
            time.sleep(.5)
            robot.set_led(0,0)
            robot.set_led(1,0)
            robot.set_musical_note(0)

    def move_up(self, event=None):
        if self.gRobotList:
            robot = self.gRobotList[1]
            self.vrobot.sl = 15
            self.vrobot.sr = 15   
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_down(self, event=None):
        if self.gRobotList:   
            robot = self.gRobotList[1]
            self.vrobot.sl = -15
            self.vrobot.sr = -15 
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_left(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[1]
            self.vrobot.sl = -6
            self.vrobot.sr = 6  
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_right(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[1]
            self.vrobot.sl = 6
            self.vrobot.sr = -6  
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr) 
            self.vrobot.t = time.time()

    def move_right_slow(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[1]
            self.vrobot.sl = 2
            self.vrobot.sr = -2
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr) 
            self.vrobot.t = time.time()  

    def move_left_slow(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[1]
            self.vrobot.sl = -2
            self.vrobot.sr = 2
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr) 
            self.vrobot.t = time.time()
            
    def stop_move(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[1]
            self.vrobot.sl = 0
            self.vrobot.sr = 0
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_to_angle_left(self, angle):
        if self.gRobotList: 
            robot = self.gRobotList[1]
            current_a = self.vrobot.a
            curr_delt_a = angle - current_a
            self.move_left()
            while (abs(curr_delt_a) > .03):
                curr_delt_a = angle - current_a
                current_a = self.vrobot.a
                time.sleep(.01)
            self.stop_move()

    def move_to_angle_right(self, angle):
        if self.gRobotList: 
            robot = self.gRobotList[1]
            current_a = self.vrobot.a
            curr_delt_a = angle - current_a
            self.move_right()
            while (abs(curr_delt_a) > .03):
                curr_delt_a = angle - current_a
                current_a = self.vrobot.a
                time.sleep(.01)
            self.stop_move()
 
    #Thread to update virtual robot based on a model of my robot
    def update_virtual_robot(self):
  
        #these values are for a charged robot 031
        #the model is based on the model provided, and if the battery is charged, meets the reqs
        noise_prox = 25 # noisy level for proximity
        noise_floor = 20 #floor ambient color - if floor is darker, set higher noise
        p_factor = 1.37 #proximity conversion - assuming linear
        d_factor = .95 #travel distance conversion (large d_factor makes vrobot slower)
        a_factor = 15.3 #rotation conversion, assuming linear (large a_factor makes vrobot slower)
        wheel_balance = -4 #value for 031. -128(L) ~ 127(R)(0: off), my hamster swerves right

        #wait until robot is connected
        while  len(self.gRobotList) < 2:
            time.sleep(0.1)

        start = True
        
        while not gVars.gQuit:
            if self.gRobotList is not None:
                #set guard robot
                self.robot = gVars.grobotList[1]   #second robot connected will be guard
                robot = self.robot
                
                #set initial position
                if (start):
                    self.vrobot.x = 0
                    self.vrobot.y =  140
                    self.vrobot.a  = pi
                    start = False
            
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
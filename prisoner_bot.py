'''
/* =======================================================================
Description:

    This file contains a prisoner robot class that has methods to calculate
    its virtual position and navigate a motionpath.
   ========================================================================*/
'''

import final_config as gVars
import final_draw as draw
from HamsterAPI.comm_ble import RobotComm
import math
import numpy as numpy
import Queue
import threading
import time 
import Tkinter as tk 

#define common angles
pi4 = 3.14159/4
pi2 = 3.14159/2
p3i4 = (3*3.14159)/4
pi = 3.14159
p5i4 = (5*3.14159)/4
p3i2 = (3*3.14159)/2
p7i4 = (7*3.14159)/4
p2i = 2 * 3.14159

class Prisoner:
    def __init__(self, comm, rCanvas):
        self.gMaxRobotNum = gVars.gMaxRobotNum
        self.gRobotList = gVars.grobotList
        self.robot = None  #first robot connected will be prisoner
        self.m = gVars.m
        self.vrobot = draw.virtual_robot()
        self.vrobot.time = time.time()
        self.localize_bool = False
        self.motionQueue = Queue.Queue() #start queue for prisoner motion
        self.motion_done = False

        #joystick commands used to help calibrate the robot model
        rCanvas.bind_all('<w>', self.move_up)
        rCanvas.bind_all('<s>', self.move_down)
        rCanvas.bind_all('<a>', self.move_left)
        rCanvas.bind_all('<d>', self.move_right)
        rCanvas.bind_all('<x>', self.stop_move)
        rCanvas.pack()
            
    # joysticking the robot 
    def move_up(self, event=None):
        if self.gRobotList:
            robot = self.gRobotList[0]
            self.vrobot.sl = 15
            self.vrobot.sr = 15   
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()
            
    def move_up_fast(self, event=None):
        if self.gRobotList:
            robot = self.gRobotList[0]
            self.vrobot.sl = 30
            self.vrobot.sr = 30 
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_down(self, event=None):
        if self.gRobotList:   
            robot = self.gRobotList[0]
            self.vrobot.sl = -15
            self.vrobot.sr = -15 
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_left(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            self.vrobot.sl = -6
            self.vrobot.sr = 6  
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_right(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            self.vrobot.sl = 6
            self.vrobot.sr = -6  
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr) 
            self.vrobot.t = time.time()
            
    def move_right_slow(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            self.vrobot.sl = 1
            self.vrobot.sr = -1
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr) 
            self.vrobot.t = time.time()  

    def move_left_slow(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            self.vrobot.sl = -1
            self.vrobot.sr = 1
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr) 
            self.vrobot.t = time.time()
            
    def stop_move(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            self.vrobot.sl = 0
            self.vrobot.sr = 0
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()
            
    def correct_left(self, prox_threshold, prev_movement):
        prox_r = self.robot.get_proximity(1)
        
        if prox_r > prox_threshold:
            self.stop_move()
            #self.move_left()
            self.robot.set_wheel(0,-1)
            self.robot.set_wheel(1,1) 
            while prox_r > (prox_threshold - 8):
                prox_r = self.robot.get_proximity(1)
                time.sleep(.01)
            self.stop_move()
            #getattr(self, prev_movement)
            self.move_up()
    
    def correct_right(self, prox_threshold, prev_movement):
        prox_l = self.robot.get_proximity(0)
        prox_r = self.robot.get_proximity(1)
        
        if prox_l > prox_threshold and prox_r < prox_threshold:
            self.stop_move()
            #self.move_left()
            self.robot.set_wheel(0,3)
            self.robot.set_wheel(1,-3) 
            while prox_l > (prox_threshold - 8):
                prox_l = self.robot.get_proximity(1)
                time.sleep(.01)
            self.stop_move()
            #getattr(self, prev_movement)
            self.move_up()

    def prox_to_dist(self, prox_l, prox_r):
        #this ensures a domain error will not be thrown when using math.log in dist calcs
        if prox_l == 0 or prox_l is None:
            prox_l = 1
        if prox_r == 0 or prox_l is None:
            prox_r = 1
        
        #convert prox value to distance in mm 
        dist_l =  (-8.035*math.log(prox_l) + 38.375) * 10
        dist_r = (-7.687*math.log(prox_r)+36.817) * 10
        
        dist = [dist_l, dist_r]
        
        return dist
    
    def check_line(self, floor_thresh):
        robot = self.robot
        left_floor = robot.get_floor(0)
        right_floor = robot.get_floor(1)
        
        if left_floor < floor_thresh and right_floor < floor_thresh:
            return True
            
    def celebrate_song(self):
        if self.gRobotList: 
            robot = self.robot
            robot.set_led(0,6)
            robot.set_led(1,6)
            robot.set_musical_note(16) #C
            time.sleep(.5)
            robot.set_musical_note(18) #D
            time.sleep(.5)
            robot.set_musical_note(20) #E
            time.sleep(.5)
            robot.set_musical_note(21) #F
            time.sleep(.5)
            robot.set_musical_note(23) #G
            time.sleep(.5)
            robot.set_musical_note(25) #A
            time.sleep(.5)
            robot.set_musical_note(27) #B
            time.sleep(.5)
            robot.set_musical_note(28) #C
            time.sleep(2)
            robot.set_musical_note(0)

    def fail_song(self):
        if self.gRobotList: 
            robot = self.robot
            robot.set_led(0,1)
            robot.set_led(1,1)
            robot.set_musical_note(17) #Db
            time.sleep(.7)
            robot.set_musical_note(16) #C
            time.sleep(.7)
            robot.set_musical_note(9) #F
            time.sleep(1.4)
            robot.set_musical_note(0)
    
    #this func calculates a motionpath for the robot
    def get_motionpath(self, vWorld, decoy_corns):
        self.vWorld = vWorld
        print "Prisoner: Calculating motion path.\n"
        
        #calc final waypoint
        decoy_edgex = decoy_corns[2]    #right edge of decoy box
        decoy_edgey = decoy_corns[3] - 20   #center of decoy box
        
        #calc waypoint 1
        wp1_x = 0
        wp1_y = decoy_corns[1] - 40 - 10
        #handle edge cases
        if wp1_y > -30:
            wp1_y = -30
        elif wp1_y < -80:
            wp1_y = - 40
            
            
        #calc waypoint 3; this is a dist_move waypoint
        wp3_x = decoy_edgex + 60
        
        #make sure the dist_move threshold is always between 30 - 80mm to get reliable measurements
        dist3_thresh = 260 - (wp3_x + 20)
        if dist3_thresh < 30:
            wp3_x = 230 - 20
            dist3_thresh = 30
        elif dist3_thresh > 80:
            wp3_x = 180 - 20
            dist3_thresh = 80
        wp3_y = wp1_y 
            
        #calc waypoint 2
        wp2_x = .5 * wp3_x #put intermediate waypoint halfway to wp3x
        wp2_y = wp1_y
        
        #calc waypoint 5
        wp5_x = wp3_x
        wp5_y = decoy_edgey
        
        #calc waypoint 3
        wp4_x = wp5_x
        wp4_y = (.5 * (wp5_y + abs(wp3_y))) + wp3_y #put intermediate waypoint halfway to wp5
        
        #make sure the dist_move threshold is always between 30 - 80mm to get reliable measurements
        dist5_thresh = 100 - (wp5_y +20)
        if dist5_thresh < 30:
            wp5_y = 70 - 20
            dist5_thresh = 30
        elif dist5_thresh > 80:
            wp5_5 = 20 - 20
            dist5_thresh = 80        
        
        #create motion path with x,y coordinates
        waypoint1 = [wp1_x, wp1_y]
        waypoint2 = [wp2_x, wp2_y]
        waypoint3 = [wp3_x, wp3_y]
        waypoint4 = [wp4_x, wp4_y]
        waypoint5 = [wp5_x, wp5_y]
 
        #add motion path to GUI
        vWorld.add_waypoint(waypoint1)
        vWorld.add_waypoint(waypoint2)
        vWorld.add_waypoint(waypoint3)
        vWorld.add_waypoint(waypoint4)
        vWorld.add_waypoint(waypoint5)
        
        #draw motion path on GUI map
        vWorld.draw_motionpath()
        
        #add motion path to Quene
        self.motionQueue.put(waypoint1)
        self.motionQueue.put(waypoint2)
        self.motionQueue.put(['dist_move', dist3_thresh, wp3_x, wp3_y, pi2]) #moves to waypoint three on map
        self.motionQueue.put(waypoint4)
        self.motionQueue.put(['dist_move', dist5_thresh, wp5_x, wp5_y, 0]) #moves to waypoint three on map
        self.motionQueue.put(['localize', 'x+', 'R']) #moves to waypoint three on map
        self.motionQueue.put(['done']) #end motionpath and return True
        
            
    #this func moves the robot to a specified waypoint (input ex: waypoint1 = [150,0])
    def xy_motion(self,waypoint):
        
        #set target waypoints
        wp_x = waypoint[0]
        wp_y = waypoint[1]
        
        #set robot current position
        current_x = self.vrobot.x
        current_y = self.vrobot.y
        current_a = self.vrobot.a % (p2i) # value aways zero to 2pi
        prox_threshold = 65

        #calc x and y distance from current to waypoint
        delt_x = round(wp_x - current_x, 2)
        delt_y = round(wp_y - current_y, 2)
                                       
        #calc angle to turn toward waypoint
        if delt_x == 0:
            if delt_y > 0:
                wp_a = 0 #go up
            elif delt_y < 0:
                wp_a = pi #go down
            elif delt_y == 0:
                print "Robot already at Waypoint"
                wp_a = current_a
        elif delt_y == 0:
            if delt_x > 0:
                wp_a = pi2 #go right
            elif delt_x < 0:
                wp_a = p3i2 #go left
            elif delt_x == 0:
                print "Robot already at Waypoint"
                wp_a = current_a                
        elif delt_x > 0:
            theta = math.atan(delt_y/delt_x)
            wp_a = pi2 - theta
        elif delt_x < 0:
            theta = math.atan(delt_y/delt_x)
            wp_a = p3i2 - theta
                
        wp_a = round(wp_a, 2) #round to two decimal places
                    
        curr_delt_a = wp_a - current_a
        delt_a = curr_delt_a
        
        #handle roll over case around 2pi
        if wp_a >= current_a:
            current_a_test = current_a + p2i
            wp_a_test = wp_a
        elif wp_a < current_a:
            current_a_test = current_a
            wp_a_test = wp_a + p2i              
            
        delt_a_test = wp_a_test - current_a_test
                
        if not wp_a == 3.14:   #when wp_a is 3.14 the waypoint is directly behind the bot, handled below
            
            #turn robot to the angle
            if delt_a > 0:
                if abs(delt_a_test) < abs(delt_a):
                    self.move_left()
                else:
                    self.move_right()    
            elif delt_a < 0:
                if abs(delt_a_test) < abs(delt_a):
                    self.move_right()
                else:
                    self.move_left()
            while abs(curr_delt_a) > .03:
                curr_delt_a = wp_a - current_a
                current_a = self.vrobot.a
                time.sleep(.001)
            self.stop_move()
        
            time.sleep(.3) #pause
            #initalize delta between current and desired waypoint
            curr_delt_x = wp_x - current_x
            curr_delt_y = wp_y - current_y
                        
            #drive to x, y waypoint coordinates. Condition looks for sign change in delta
            if p7i4 <= wp_a <= p2i or 0 <= wp_a <= pi4 or p3i4 <= wp_a <= p5i4: #if driving mainly up or down
                self.move_up()
                while numpy.sign(curr_delt_y) == numpy.sign(delt_y):
                    curr_delt_x = wp_x - self.vrobot.x
                    curr_delt_y = wp_y - self.vrobot.y
                    time.sleep(.001)
            elif pi4 < wp_a < p3i4 or p5i4 <= wp_a <= p7i4: #if driving mainly left or right
                self.move_up()
                while numpy.sign(curr_delt_x) == numpy.sign(delt_x):
                    curr_delt_x = wp_x - self.vrobot.x
                    curr_delt_y = wp_y - self.vrobot.y
                    time.sleep(.001)                    
            self.stop_move()
        #move backward case
        else:
            time.sleep(.5) #pause
            #initalize delta between current and desired waypoint
            curr_delt_x = wp_x - current_x
            curr_delt_y = wp_y - current_y
                                
            #drive to x, y waypoint coordinates. Condition looks for sign change in delta
            self.move_down()
            while numpy.sign(curr_delt_y) == numpy.sign(delt_y):
                curr_delt_x = wp_x - self.vrobot.x
                curr_delt_y = wp_y - self.vrobot.y                    
                time.sleep(.001)
            self.stop_move()
            
        self.motionQueue.task_done()
        time.sleep(.3)
        

    
    #this func drives the robot until a distance (input ex: queue_item = ['dist_move', dist thres, vfinal x, vfinal y, vfinal a])       
    def dist_move(self, queue_item):
        if self.robot:
            robot = self.robot
            
            dist_thresh = queue_item[1]
            
            #check proximity sensors 
            prox_l = robot.get_proximity(0)
            prox_r = robot.get_proximity(1)
            
            #convert prox value to distance in mm 
            dist_l =  self.prox_to_dist(prox_l, prox_r)[0]
            dist_r = self.prox_to_dist(prox_l, prox_r)[1]
                      
            #move forward for set time
            self.move_up()
            while dist_l >= dist_thresh or dist_r >= dist_thresh:
                #update proximity sensors 
                prox_l = robot.get_proximity(0)
                prox_r = robot.get_proximity(1)
             
                #convert prox value to distance in mm 
                dist_l =  self.prox_to_dist(prox_l, prox_r)[0]
                dist_r = self.prox_to_dist(prox_l, prox_r)[1]
                
                if dist_l < 80 and dist_r < 80: #allow robot to drive close to wall before correcting
                    if dist_r > dist_l:
                        self.stop_move()
                        while not (dist_r - 2) < dist_l < (dist_r + 2):
                            robot.set_wheel(0,0)
                            robot.set_wheel(1,3)
            
                            #update proximity sensors 
                            prox_l = robot.get_proximity(0)
                            prox_r = robot.get_proximity(1)
                         
                            #convert prox value to distance in mm 
                            dist_l =  self.prox_to_dist(prox_l, prox_r)[0]
                            dist_r = self.prox_to_dist(prox_l, prox_r)[1]
                                  
                            time.sleep(0.01)
                        self.move_up()    
                    elif dist_l < dist_r:
                        self.stop_move()
                        while not (dist_l - 2) < dist_r < (dist_l + 2):
                            robot.set_wheel(0,3)
                            robot.set_wheel(1,0)
            
                            #update proximity sensors 
                            prox_l = robot.get_proximity(0)
                            prox_r = robot.get_proximity(1)
                         
                            #convert prox value to distance in mm 
                            dist_l =  self.prox_to_dist(prox_l, prox_r)[0]
                            dist_r = self.prox_to_dist(prox_l, prox_r)[1]
                                  
                            time.sleep(0.01)
                        self.move_up()
            self.stop_move()
            #localize bot
            self.vrobot.x = queue_item[2]
            self.vrobot.y = queue_item[3]
            self.vrobot.a = queue_item[4]
            
            self.motionQueue.task_done()
            time.sleep(.3)


    #this func localizes the robot relative to a specifed object (input ex: queue_item = ['localize', 'y-', 'F'])          
    def localize(self, queue_item):
            if self.gRobotList:
                robot = self.robot
                
                #record starting x,y for use in offset calcs below
                x0 = self.vrobot.x
                y0 = self.vrobot.y
                
                loc_axis = queue_item[1]
                loc_obj = queue_item[2]
                
                #define angles for given axis
                if loc_axis == 'x+':
                    loc_angle =  pi2
                elif loc_axis == 'y+':
                    loc_angle = p2i
                elif loc_axis == 'x-':
                    loc_angle = p3i2
                elif loc_axis == 'y-':
                    loc_angle = pi
                
                #set current robot angle    
                current_a = self.vrobot.a 
                
                curr_delt_a = loc_angle - current_a
                delt_a = loc_angle - current_a

                #handle roll over case around 2pi
                if loc_angle >= current_a:
                    current_a_test = current_a + p2i
                    loc_angle_test = loc_angle
                elif loc_angle < current_a:
                    current_a_test = current_a
                    loc_angle_test = wp_a + p2i              
                    
                delt_a_test = loc_angle_test - current_a_test
        
                #turn robot to the angle
                if delt_a > 0:
                    if abs(delt_a_test) < abs(delt_a):
                        self.move_left()
                    else:
                        self.move_right()    
                elif delt_a < 0:
                    if abs(delt_a_test) < abs(delt_a):
                        self.move_right()
                    else:
                        self.move_left()
                while abs(curr_delt_a) > .03:
                    curr_delt_a = loc_angle - current_a
                    current_a = self.vrobot.a
                    time.sleep(.01)
                self.stop_move()                
                
                #check proximity sensors 
                prox_l = robot.get_proximity(0)
                prox_r = robot.get_proximity(1)
    
                #initialize lists used to calc a moving average for sensor values
                dist_l_list = [0, 0, 0]
                dist_r_list = [100, 100, 100]
                
                #align robot perpendicular to the box and calc y value for robot
                ave_dist_l = 0
                ave_dist_r = 100   #dummy variables
                tol = 1.5 #tolerance value in mm
                max_dist = 190 #minimum value to ensure sensors are not seeing nothing
                
                #Turn and localize  
                #angled to the right case
                if prox_r <= prox_l:
                    self.move_left_slow()
                    #turn until sensor values are equal
                    while not ave_dist_r - tol < ave_dist_l < ave_dist_r + tol or ave_dist_l > max_dist:
                        
                        #this ensures a domain error will not be thrown when using math.log in dist calcs
                        if prox_l == 0 or prox_l is None:
                            prox_l = 1
                        if prox_r == 0 or prox_l is None:
                            prox_r = 1
                        
                        #calc distance again and add it to the list   
                        dist_l =  (-8.035*math.log(prox_l) + 38.375) * 10
                        dist_r = (-7.687*math.log(prox_r)+36.817) * 10
                        dist_l_list.append(dist_l)
                        dist_r_list.append(dist_r)
                        
                        #calc moving average
                        if len(dist_l_list) >= 2:
                            ave_dist_l = sum(dist_l_list[-2:])/2
                            ave_dist_r = sum(dist_r_list[-2:])/2
                        
                        #turn slowly and update prox variables
                        
                        prox_l = robot.get_proximity(0)
                        prox_r = robot.get_proximity(1)
                        time.sleep(.001)
                                                
                #angled to the left case        
                elif prox_l < prox_r:
                    while not ave_dist_l - tol < ave_dist_r < ave_dist_l + tol or ave_dist_l > max_dist:
                        self.move_right_slow()
                        # this ensures a domain error will not be thrown when using math.log in dist calcs
                        if prox_l == 0 or prox_l is None:
                            prox_l = 1
                        if prox_r == 0 or prox_r is None:
                            prox_r = 1
                            
                        #calc distance again and add it to the list   
                        dist_l =  (-8.035*math.log(prox_l) + 38.375) * 10 #log fit to empiracle data [mm]
                        dist_r = (-7.687*math.log(prox_r)+36.817) * 10
                        dist_l_list.append(dist_l)
                        dist_r_list.append(dist_r)
                        
                        #calc moving average
                        if len(dist_l_list) >= 2:
                            ave_dist_l = sum(dist_l_list[-2:])/2
                            ave_dist_r = sum(dist_r_list[-2:])/2
                        
                        #turn slowly and update prox variables
                        
                        prox_l = robot.get_proximity(0)
                        prox_r = robot.get_proximity(1)
                        time.sleep(.001)
                                                
                #stop turning        
                self.stop_move()
                
                #calc offset 
                offset = ave_dist_l + 20 #add 20 mm to get center point of robot
                
                #set absolute reference point values for each case
                if  loc_axis == 'x+' and loc_obj == 'R': 
                    self.vrobot.x = 260 - offset
                    self.vrobot.y = y0
                    self.vrobot.a = pi2
                
                self.motionQueue.task_done()
                time.sleep(.3)
                
    
    #this function handles the input from the queue to guide the robot            
    def move_to_waypoint(self):
        #pause to allow robot to connect before beginning

        while (self.motionQueue.qsize() > 0):
            if self.gRobotList:
                robot = self.robot
                queue_item = self.motionQueue.get(True)
                
                #look at queue items
                if queue_item[0] == 'localize':
                    self.localize(queue_item)   #localize the robot
                elif queue_item[0] == 'dist_move':
                    self.dist_move(queue_item)   #move until a certain distance from wall
                elif queue_item[0] == 'done':
                    self.motion_done = True
                    
                else:
                    self.xy_motion(queue_item) #go to the x,y coordinate
            time.sleep(.01)
        
        "Prisoner: Motionpath navigatied!"
        #return True
    
    #this pushes the decoy box into the target zone   
    def push_decoy(self, vWorld):
        
        robot = self.gRobotList[0]       
        
        #check proximity sensors 
        prox_l = robot.get_proximity(0)
        prox_r = robot.get_proximity(1)

        #initialize lists used to calc a moving average for sensor values
        dist_l_list = [0, 0, 0]
        dist_r_list = [100, 100, 100]
        
        #turn robot toward decoy boundary box
        init_rot = p5i4                
        current_a = self.vrobot.a        
        curr_delt_a = init_rot - current_a
        self.move_right()  
        while abs(curr_delt_a) > .03:
            curr_delt_a = init_rot - current_a
            current_a = self.vrobot.a
            time.sleep(.01)
        self.vrobot.a = p5i4
        self.stop_move()            
            
        #set variables
        ave_dist_l = 0
        ave_dist_r = 100   #dummy variables
        tol = 2 #tolerance value in mm
        max_dist = 75 #minimum value to ensure sensors are not seeing nothing
        
        #Turn and find decoy
        self.move_right()
        #turn until sensor values are equal
        while not ave_dist_r - tol < ave_dist_l < ave_dist_r + tol or ave_dist_l > max_dist:
            
            #calc distance again and add it to the list   
            dist_l =  self.prox_to_dist(prox_l, prox_r)[0]
            dist_r = self.prox_to_dist(prox_l, prox_r)[1]
            dist_l_list.append(dist_l)
            dist_r_list.append(dist_r)
            
            #calc moving average
            if len(dist_l_list) >= 2:
                ave_dist_l = sum(dist_l_list[-2:])/2
                ave_dist_r = sum(dist_r_list[-2:])/2
            
            #turn slowly and update prox variables
            
            prox_l = robot.get_proximity(0)
            prox_r = robot.get_proximity(1)
            time.sleep(.01)
            
        self.stop_move() 
        
        
        print "Prisoner: Decoy box located. \n"
        time.sleep(.3)


        #push box to target zone
        floor_thresh = 60
        push_timer = time.time() + 20 #timeout if doesnt' find line
        self.move_up()
        while not self.check_line(floor_thresh) and time.time() < push_timer:
            time.sleep(0.001)
        self.stop_move() 

        #update decoy box location
        box_x1 = self.vrobot.x - 20.8 - 40 # .8 accounts for psd arm length
        box_y1 = self.vrobot.y - 20
        box_x2 = box_x1 + 40    #top right
        box_y2 = box_y1 + 40
        
        decoy_pos = [box_x1, box_y1, box_x2, box_y2]
        vWorld.add_decoy(decoy_pos)
        vWorld.draw_decoy()
        print "Prisoner: Decoy box moved to target zone \n"
        time.sleep(.3)

    #Thread to update virtual robot based on a model of my robot
    def update_virtual_robot(self):
  
        #these values are for a charged robot 031
        #the model is based on the model provided, and if the battery is charged, meets the reqs
        noise_prox = 25 # noisy level for proximity
        noise_floor = 20 #floor ambient color - if floor is darker, set higher noise
        p_factor = 1.4 #proximity conversion - assuming linear
        d_factor = 1.1 #travel distance conversion (large d_factor makes vrobot slower)
        a_factor = 15 #rotation conversion, assuming linear (large a_factor makes vrobot slower)
        wheel_balance = -6 #value for 031. -128(L) ~ 127(R)(0: off), my hamster swerves right

        #wait until robot is connected
        while not self.gRobotList:
            time.sleep(0.1)
        
        while not gVars.gQuit:
            if self.gRobotList is not None:
                #set prisoner robot
                self.robot = gVars.grobotList[0]   #first robot connected will be prisoner
                robot = self.gRobotList[0]
            
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
                    self.vrobot.a = (self.vrobot.a + (self.vrobot.sl * del_t)/a_factor) % (p2i) #always have angle between 0 and 2pi
                    
                    
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
                    

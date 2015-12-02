
import Tkinter as tk 
import time 
from HamsterAPI.comm_ble import RobotComm
import math
import numpy as numpy
import threading
import final_draw as draw

UPDATE_INTERVAL = 100

#define common angles
pi4 = 3.14159/2
pi2 = 3.14159/2
p3i4 = (3*3.14159)/4
pi = 3.14159
p5i4 = (5*3.14159)/4
p3i2 = (3*3.14159)/2
p7i4 = (7*3.14159)/4
p2i = 2 * 3.14159

gMaxRobotNum = 1; # max number of robots to control
gQuit = False
m = None

class VirtualWorldGui:
    def __init__(self, vWorld, m):
        self.vworld = vWorld
        
        #initialize grid, map, and motionpath on
        self.drawGrid()
        self.drawMap(None)
        self.drawMotionpath()
        
        #create GUI buttons
        self.button0 = tk.Button(m,text="Grid")
        self.button0.pack(side='left')
        self.button0.bind('<Button-1>', self.drawGrid)

        self.button1 = tk.Button(m,text="Clear")
        self.button1.pack(side='left')
        self.button1.bind('<Button-1>', self.clearCanvas)

        self.button2 = tk.Button(m,text="Reset")
        self.button2.pack(side='left')
        self.button2.bind('<Button-1>', self.resetvRobot)

        self.button3 = tk.Button(m,text="Map")
        self.button3.pack(side='left')
        self.button3.bind('<Button-1>', self.drawMap)

        self.button4 = tk.Button(m,text="Trace")
        self.button4.pack(side='left')
        self.button4.bind('<Button-1>', self.toggleTrace)

        self.button5 = tk.Button(m,text="Prox Dots")
        self.button5.pack(side='left')
        self.button5.bind('<Button-1>', self.toggleProx)

        self.button6 = tk.Button(m,text="Floor Dots")
        self.button6.pack(side='left')
        self.button6.bind('<Button-1>', self.toggleFloor)

        self.button7 = tk.Button(m,text="Motion Path")
        self.button7.pack(side='left')
        self.button7.bind('<Button-1>', self.drawMotionpath)

        self.button8 = tk.Button(m,text="Exit")
        self.button8.pack(side='left')
        self.button8.bind('<Button-1>', stopProg)
        
    def resetvRobot(self, event=None):
        self.vworld.vrobot.reset_robot()

    def toggleTrace(self, event=None):
        if self.vworld.trace:
            self.vworld.trace = False
            self.button4["text"] = "Trace"
        else:
            self.vworld.trace = True
            self.button4["text"] = "No Trace"

    def toggleProx(self, event=None):
        if self.vworld.prox_dots:
            self.vworld.prox_dots = False
            self.button5["text"] = "Prox Dots"
        else:
            self.vworld.prox_dots = True
            self.button5["text"] = "No Prox Dots"

    def toggleFloor(self, event=None):
        if self.vworld.floor_dots:
            self.vworld.floor_dots = False
            self.button6["text"] = "Floor Dots"
        else:
            self.vworld.floor_dots = True
            self.button6["text"] = "No Floor Dots"

    def drawMap(self, event=None):
        self.vworld.draw_map(None)
        
    def drawMotionpath(self, event=None):
        self.vworld.draw_motionpath()

    def drawGrid(self, event=None):
        x1, y1 = 0, 0
        x2, y2 = self.vworld.canvas_width*2, self.vworld.canvas_height*2
        del_x, del_y = 20, 20
        num_x, num_y = x2 / del_x, y2 / del_y
        # draw center (0,0)
        self.vworld.canvas.create_rectangle(self.vworld.canvas_width-3,self.vworld.canvas_height-3,
                self.vworld.canvas_width+3,self.vworld.canvas_height+3, fill="red")
        # horizontal grid
        for i in range(num_y):
            y = i * del_y
            self.vworld.canvas.create_line(x1, y, x2, y, fill="yellow")
        # verticle grid
        for j in range(num_x):
            x = j * del_x
            self.vworld.canvas.create_line(x, y1, x, y2, fill="yellow")

    def clearCanvas(self, event=None):
        vcanvas = self.vworld.canvas
        vrobot = self.vworld.vrobot
        vcanvas.delete("all")
        poly_points = [0,0,0,0,0,0,0,0]
        vrobot.poly_id = vcanvas.create_polygon(poly_points, fill='blue')
        vrobot.prox_l_id = vcanvas.create_line(0,0,0,0, fill="red")
        vrobot.prox_r_id = vcanvas.create_line(0,0,0,0, fill="red")
        vrobot.floor_l_id = vcanvas.create_oval(0,0,0,0, outline="white", fill="white")
        vrobot.floor_r_id = vcanvas.create_oval(0,0,0,0, outline="white", fill="white")

    def updateCanvas(self, drawQueue):
        while (drawQueue.qsize() > 0):
            drawData = drawQueue.get()
            self.vworld.canvas.coords(drawData[0], drawData[1])
        self.vworld.canvas.after(UPDATE_INTERVAL, self.updateCanvas, drawQueue)
   
class MotionPath:
    def __init__(self, comm, m, rCanvas, motionQueue):
        self.gMaxRobotNum = 1
        self.gRobotList = comm.robotList
        self.m = m
        self.vrobot = draw.virtual_robot()
        self.vrobot.time = time.time()
        self.localize_bool = False
        self.motionQueue = motionQueue

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
            
    def celebrate_music(self):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            robot.set_musical_note(80)
            time.sleep(.7)
            robot.set_musical_note(100)
            time.sleep(.7)
            robot.set_musical_note(0)
    
    #this func moves the robot to a specified waypoint (input ex: waypoint1 = [150,0])
    def xy_motion(self,waypoint):
                #set target waypoints
                wp_x = waypoint[0]
                wp_y = waypoint[1]
                
                #set robot current position
                current_x = self.vrobot.x
                current_y = self.vrobot.y
                current_a = self.vrobot.a % (2 * 3.1415) # value aways zero to 2pi
                
                #print to terminal
                print "Moving to waypoint(", wp_x,"," , wp_y, ").\n\n"

                #calc x and y distance from curren to waypoint
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
                    if theta > 0:
                        wp_a = pi2 - theta
                    if theta < 0:
                        wp_a = pi2 + (-theta)
                elif delt_x < 0:
                    theta = math.atan(delt_y/delt_x)
                    if theta > 0:
                        wp_a = p3i2 - theta
                    if theta < 0:
                        wp_a = p3i2 + (-theta)
                
                wp_a = round(wp_a, 2) #round to two decimal places
                #handle rollover cases around 2pi
                if wp_a < pi2:
                    wp_a = wp_a + p2i
                if current_a < pi2:
                    current_a = current_a + p2i    
                    
                curr_delt_a = wp_a - current_a
                delt_a = wp_a - current_a
                                        
                #turn robot to the angle
                while abs(curr_delt_a) > .05:
                    curr_delt_a = wp_a - current_a
                    if delt_a > 0:
                        self.move_right()
                    elif delt_a < 0:
                        self.move_left()
                    current_a = self.vrobot.a 
                    time.sleep(.001)
                self.stop_move()
                
                time.sleep(.5) #pause
                #initalize delta between current and desired waypoint
                curr_delt_x = wp_x - current_x
                curr_delt_y = wp_y - current_y
                
                wp_a = wp_a % (2 * 3.1415) # value aways zero to 2pi
                
                #drive to x, y waypoint coordinates. Condition looks for sign change in delta
                if p7i4 <= wp_a <= p2i or 0 <= wp_a <= pi4 or p3i4 <= wp_a <= p5i4: #if driving mainly up or down
                    while numpy.sign(curr_delt_y) == numpy.sign(delt_y):
                        curr_delt_x = wp_x - self.vrobot.x
                        curr_delt_y = wp_y - self.vrobot.y                    
                        self.move_up()
                        curr_delt_x = wp_x - self.vrobot.x
                        curr_delt_y = wp_y - self.vrobot.y
                        time.sleep(.001)
                elif pi4 < wp_a < p3i4 or p5i4 <= wp_a <= p7i4: #if driving mainly left or right
                    while numpy.sign(curr_delt_x) == numpy.sign(delt_x):
                        curr_delt_x = wp_x - self.vrobot.x
                        curr_delt_y = wp_y - self.vrobot.y                    
                        self.move_up()
                        curr_delt_x = wp_x - self.vrobot.x
                        curr_delt_y = wp_y - self.vrobot.y
                        time.sleep(.001)                    
                self.stop_move()
            
                self.motionQueue.task_done()
                time.sleep(1) #pause
                
    #this func localizes the robot relative to a specifed object (input ex: queue_item = ['localize', 'y-', 'F'])          
    def localize(self, queue_item):
            if self.gRobotList:
                robot = self.gRobotList[0]
                
                #record starting x,y for use in offset calcs below
                x0 = self.vrobot.x
                y0 = self.vrobot.y
                
                loc_axis = queue_item[1]
                loc_obj = queue_item[2]
                
                #print update to terminal
                print "Localizing robot relative to ", loc_obj, ".\n\n"
                
                #define angles for given axis
                if loc_axis == 'x+':
                    loc_angle =  p3i2
                elif loc_axis == 'y+':
                    loc_angle = p2i
                elif loc_axis == 'x-':
                    loc_angle = pi2
                elif loc_axis == 'y-':
                    loc_angle = pi
                
                #set current robot angle    
                current_a = self.vrobot.a % (2 * 3.1415) # value aways zero to 2pi

                curr_delt_a = loc_angle - current_a
                delt_a = loc_angle - current_a                
                #turn robot to the angle
                while abs(curr_delt_a) > .2:
                    curr_delt_a = loc_angle - current_a
                    if delt_a > 0:
                        self.move_right()
                    elif delt_a < 0:
                        self.move_left()
                    current_a = self.vrobot.a % (2 * 3.1415) # value aways zero to 2pi
                    time.sleep(.001)
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
                tol = 2 #tolerance value in mm
                max_dist = 190 #minimum value to ensure sensors are not seeing nothing
                
                #handle special cases
                if loc_axis == 'y-' and loc_obj == 'F':
                    prox_r = 0    #always turns left when localizing to F
                    tol = 4
                    max_dist = 110 #make sure robot is seeing an object
                    
                #Turn and localize  
                #angled to the right case
                if prox_r <= prox_l:
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
                        self.move_left_slow()
                        prox_l = robot.get_proximity(0)
                        prox_r = robot.get_proximity(1)
                        time.sleep(.001)
                                                
                #angled to the left case        
                elif prox_l < prox_r:
                    while not ave_dist_l - tol < ave_dist_r < ave_dist_l + tol or ave_dist_l > max_dist:
                        
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
                        self.move_right_slow()
                        prox_l = robot.get_proximity(0)
                        prox_r = robot.get_proximity(1)
                        time.sleep(.001)
                                                
                #stop turning        
                self.stop_move()
                
                #calc offset 
                offset = ave_dist_l + 20 #add 20 mm to get center point of robot
                
                #set absolute reference point values for each case
                if  loc_axis == 'x+' and loc_obj == 'F': 
                    self.vrobot.x = 40 + offset
                    self.vrobot.y = y0
                    self.vrobot.a = p3i2
                elif  loc_axis == 'y-' and loc_obj == 'F': 
                    self.vrobot.x = x0
                    self.vrobot.y = 50 + offset
                    self.vrobot.a = pi + .15 #waypoint is slightly offset
                elif loc_axis == 'x+' and loc_obj == 'A': #Goal Case
                    if self.vrobot.a >= p3i2: #robot angled right
                        #calc x,y coordinates
                        fin_angle = self.vrobot.a - p3i2
                        x_off = (offset*math.cos(fin_angle))
                        y_off = (offset*math.sin(fin_angle))
                        self.vrobot.x = -220 + x_off
                        self.vrobot.y = - y_off
                    elif self.vrobot.a < p3i2: #robot angled left
                        #calc x,y coordinates
                        fin_angle =  p3i2 - self.vrobot.a 
                        x_off = (offset*math.cos(fin_angle))
                        y_off = (offset*math.sin(fin_angle))
                        self.vrobot.x = -220 + x_off
                        self.vrobot.y = y_off
                    
                    #check to see if goal was found
                    if ave_dist_l < 150:
                        #make sure robot is perpendicular to goal
                        if not -10 < self.vrobot.y < 10:
                            print "Attempting to align to goal...\n\n"
                            self.motionQueue.put(['time_move', 'x-', 1])
                            self.motionQueue.put([-165, 0])
                            self.motionQueue.put([-170, 0])
                            self.motionQueue.put(['localize', 'x+', 'A'])
                        else:
                            print "We did it! Facing the GOAL!"
                            self.celebrate_music()
   
                    elif ave_dist_l >+ 150:
                        endtime = time.time() + 8 #timeout if goal not found
                        #turn until sensor values are equal
                        while not ave_dist_r - 5 < ave_dist_l < ave_dist_r + 5 or ave_dist_l > 130:
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
                            self.move_left_slow()
                            prox_l = robot.get_proximity(0)
                            prox_r = robot.get_proximity(1)
                            time.sleep(.01)
                            
                            #recalc abs position
                            if self.vrobot.a >= p3i2: #robot angled right
                                #calc x,y coordinates
                                fin_angle = self.vrobot.a - p3i2
                                x_off = (offset*math.cos(fin_angle))
                                y_off = (offset*math.sin(fin_angle))
                                self.vrobot.x = -220 + x_off
                                self.vrobot.y = - y_off
                            elif self.vrobot.a < p3i2: #robot angled left
                                #calc x,y coordinates
                                fin_angle =  p3i2 - self.vrobot.a 
                                x_off = (offset*math.cos(fin_angle))
                                y_off = (offset*math.sin(fin_angle))
                                self.vrobot.x = -220 + x_off
                                self.vrobot.y = y_off
                            
                            #if timeout occurs assume robot is not at goal
                            if endtime < time.time():
                                print "G-dang, I didn't find the goal. Recharge hamster and try again."
                                break
                            
                        if ave_dist_l < 150:
                            #make sure robot is perpendicular to goal
                            if not -10 < self.vrobot.y < 10:
                                print "Attempting to align to goal...\n\n"
                                self.motionQueue.put(['time_move', 'x-', 1])
                                self.motionQueue.put([-165, 0])
                                self.motionQueue.put([-170, 0])
                                self.motionQueue.put(['localize', 'x+', 'A'])
                            else:
                                print "We did it! Facing the GOAL!"
                                self.celebrate_music()
                self.stop_move()            
                self.motionQueue.task_done()
                time.sleep(1) #pause
                
    #this func drives the robot for a specified number of seconds (input ex: queue_item = ['time_move', 'x+', 4])              
    def time_move(self, queue_item):       
            if self.gRobotList:
                robot = self.gRobotList[0]
                
                dist_axis = queue_item[1]
                move_time = queue_item[2]
                
                #output update to terminal 
                print "Driving for ", move_time, " seconds in ", dist_axis, " direction.\n\n"
                
                #check proximity sensors 
                prox_l = robot.get_proximity(0)
                prox_r = robot.get_proximity(1)
                #move forward for set time
                endtime = time.time() + move_time
                while time.time() < endtime:
                    if dist_axis == 'x+':
                        self.move_up()
                    elif dist_axis == 'x-':
                        self.move_down()
                    time.sleep(.01)   
                    #stop         
                    self.stop_move()
                
                self.motionQueue.task_done()
                time.sleep(1) #pause
    
    #this function handles the input from the queue to guide the robot            
    def move_to_waypoint(self):
        #pause to allow robot to connect before beginning
        time.sleep(5)

        while (self.motionQueue.qsize() > 0):
            if self.gRobotList:
                robot = self.gRobotList[0]
                queue_item = self.motionQueue.get(True)
                
                #look at queue items
                if queue_item[0] == 'localize':
                    self.localize(queue_item)   #localize the robot
                elif queue_item[0] == 'time_move':
                    self.time_move(queue_item)  #move for a set amount of time
                else:
                    self.xy_motion(queue_item) #go to the x,y coordinate
            time.sleep(.01)
                
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
        while not self.gRobotList:
            print "waiting for robot to connect"
            time.sleep(0.1)

        print "Connected to robot!"
        
        while not gQuit:
            if self.gRobotList is not None:
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
                    
def stopProg(event=None):
    global gQuit
    global m
    #m.quit()
    gQuit = True
    print "Exit"

#Thread to draw the robot in the GUI
def draw_virtual_world(virtual_world, motionpath):
    time.sleep(1) # give time for robot to connect.
    while not gQuit:
        if motionpath.gRobotList is not None:
            virtual_world.draw_robot()
            virtual_world.draw_prox("left")
            virtual_world.draw_prox("right")
            virtual_world.draw_floor("left")
            virtual_world.draw_floor("right")
        time.sleep(0.1)

#def main(argv=None): 
#    global m
#    comm = RobotComm(gMaxRobotNum)
#    comm.start()
#    print 'Bluetooth starts'
#    m = tk.Tk() #root
#    drawQueue = Queue.Queue(0)
#    motionQueue = Queue.Queue()
#
#    #creating the virtual appearance of the robot
#    canvas_width = 700 # half width
#    canvas_height = 300  # half height
#    rCanvas = tk.Canvas(m, bg="white", width=canvas_width*2, height=canvas_height*2)
#
#    motionpath = MotionPath(comm, m, rCanvas, motionQueue)
#    
#    # visual elements of the virtual robot 
#    poly_points = [0,0,0,0,0,0,0,0]
#    motionpath.vrobot.poly_id = rCanvas.create_polygon(poly_points, fill='blue') #robot
#    motionpath.vrobot.prox_l_id = rCanvas.create_line(0,0,0,0, fill="red") #prox sensors
#    motionpath.vrobot.prox_r_id = rCanvas.create_line(0,0,0,0, fill="red")
#    motionpath.vrobot.floor_l_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white") #floor sensors
#    motionpath.vrobot.floor_r_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white")
#
#    time.sleep(1)
#
#    update_vrobot_thread = threading.Thread(target=motionpath.update_virtual_robot)
#    update_vrobot_thread.daemon = True
#    update_vrobot_thread.start()
#    
#    #create the virtual worlds that contains the virtual robot
#    vWorld = virtual_world(drawQueue, motionpath.vrobot, rCanvas, canvas_width, canvas_height)
#    
#    #create motion path with x,y coordinates
#    waypoint1 = [150,0]
#    waypoint2 = [106,40]
#    waypoint3 = [63,80]
#    waypoint4 = [30,110]
#    waypoint5 = [5, 110]
#    waypoint6 = [-20, 105]
#    waypoint7 = [-50, 80]
#    waypoint8 = [-70, 40]
#    waypoint9 = [-95,5]
#    waypoint10 = [-140, 0]
#    waypoint11 = [-160,0]
#    
#    #add motion path to GUI
#    vWorld.add_waypoint(waypoint1)
#    vWorld.add_waypoint(waypoint2)
#    vWorld.add_waypoint(waypoint3)
#    vWorld.add_waypoint(waypoint4)
#    vWorld.add_waypoint(waypoint5)
#    vWorld.add_waypoint(waypoint6)
#    vWorld.add_waypoint(waypoint7)
#    vWorld.add_waypoint(waypoint8)
#    vWorld.add_waypoint(waypoint9)
#    vWorld.add_waypoint(waypoint10)
#    vWorld.add_waypoint(waypoint11)
#
#    #add motion path to Quene
#    motionQueue.put(waypoint1)
#    motionQueue.put(['localize', 'x+', 'F'])
#    motionQueue.put(waypoint2)
#    motionQueue.put(waypoint3)
#    motionQueue.put(waypoint4)
#    motionQueue.put(['localize', 'y-', 'F'])
#    motionQueue.put(waypoint5)
#    motionQueue.put(waypoint6)
#    motionQueue.put(waypoint7)
#    motionQueue.put(waypoint8)
#    motionQueue.put(waypoint9)
#    motionQueue.put(waypoint10)
#    motionQueue.put(waypoint11)
#    motionQueue.put(['localize', 'x+', 'A'])
#    
#    #start a thread to navigate the set path
#    waypoint_thread = threading.Thread(target=motionpath.move_to_waypoint)
#    waypoint_thread.daemon = True
#    waypoint_thread.start()
#        
#    #objects in the world
#    rect1 = [-260, -20, -220, 20]  #A
#    rect2 = [-140, 80, -100, 180]  #B
#    rect3 = [-100, 140, 0, 180]    #C
#    rect4 = [-140, -180, -100, -80] #D
#    rect5 = [-100, -180, 0, -140]    #E
#    rect6 = [0, -50, 40, 50] #F
#    vWorld.add_obstacle(rect1)
#    vWorld.add_obstacle(rect2)
#    vWorld.add_obstacle(rect3)
#    vWorld.add_obstacle(rect4)
#    vWorld.add_obstacle(rect5)
#    vWorld.add_obstacle(rect6)
#
#    draw_world_thread = threading.Thread(target=draw_virtual_world, args=(vWorld, motionpath))
#    draw_world_thread.daemon = True
#    draw_world_thread.start()
#
#    gui = VirtualWorldGui(vWorld, m)
#
#    rCanvas.after(200, gui.updateCanvas, drawQueue)
#    m.mainloop()
#
#    for robot in motionpath.gRobotList:
#        robot.reset()
#    comm.stop()
#    comm.join()
#
#if __name__ == "__main__":
#    main()
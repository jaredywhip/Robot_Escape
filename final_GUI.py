import Tkinter as tk 
import time 
from HamsterAPI.comm_ble import RobotComm
import math
import numpy as numpy
import threading
import final_draw as draw

UPDATE_INTERVAL = 100

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
            time.sleep(0.01)
                    

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

'''
/* =======================================================================
Description:

    This file contains a Virtual World Gui class, a function to draw the
    virtual world, and a function to exit the program.
   ========================================================================*/
'''

import final_config as gVars
import final_draw as draw
import time
import Tkinter as tk 
 
UPDATE_INTERVAL = 100

class VirtualWorldGui:
    def __init__(self, vWorld, m):
        self.vworld = vWorld
        
        #initialize grid, map, and motionpath on
        self.drawGrid()
        self.drawMap(None)
        self.drawBoundary()
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

        self.button8 = tk.Button(m,text="Exit", command=stopProg)
        self.button8.pack(side='left')
        self.button8.bind('<Button-1>')
        
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
        
    def drawBoundary(self):
        self.vworld.draw_boundary()
    
    def drawDecoy(self):
        self.vworld.draw_decoy()
        
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
   
#Function to end the program
def stopProg():
    gVars.m.destroy()
    gVars.gQuit = True
    print "Exit"
    
#Thread to draw the robot in the GUI
def draw_virtual_world(virtual_world, prisoner, guard):
    time.sleep(1) # give time for robot to connect.
    while not gVars.gQuit:
        #draw prisoner robot
        if prisoner.robot is not None:
            virtual_world.draw_pris_robot()
            virtual_world.draw_pris_prox("left")
            virtual_world.draw_pris_prox("right")
            virtual_world.draw_pris_floor("left")
            virtual_world.draw_pris_floor("right")
        #draw guard robot
        if guard.robot is not None:
            virtual_world.draw_guard_robot()
            virtual_world.draw_guard_prox("left")
            virtual_world.draw_guard_prox("right")
            virtual_world.draw_guard_floor("left")
            virtual_world.draw_guard_floor("right")
        time.sleep(0.1)

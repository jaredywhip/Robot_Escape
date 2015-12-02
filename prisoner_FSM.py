import time
import threading
import Tkinter as tk 
import Queue
import random
from HamsterAPI.comm_ble import RobotComm
import final_GUI as gui
import prisoner_motionpath as pris_mp
import prisoner_scan as pris_scan
#from final_settings import *#file that contains global variables

#gQuit = False
#gMaxRobotNum = 2; # max number of robots to control
#event_queue = None
#m = None

  
class Event:
  def __init__(self, name):
    self.name = name


class State:
    def __init__(self):
      self.name = ''
      self.transitions = {}

    def add_transition(self, toState, callback):
      self.transitions[toState] = callback

class EventFsm:
    def __init__(self):
        self.states = {}
        self.startState = None
        self.currentState = None

    def add_state(self, name):
        a_state = State()
        a_state.name = name
        self.states[name] = a_state
        return a_state

    def set_start(self, name):
        self.startState = name

    def set_current(self, name):
        self.currentState = name

    #thread to run FSM
    def run(self):
      while not gQuit:
        if event_queue:
          event = event_queue.get()
          print 'event', event.name
          if event.name in self.states[self.currentState].transitions:
            self.states[self.currentState].transitions[event.name]()
            self.set_current(event.name)
        time.sleep(1)

def init_to_scan(vWorld):
  #print pris_fsm.currentState
  print pris_fsm.currentState
  print "transition init -> scan"
  
  #start scanning
  scan_result = []
  #while pris_fsm.currentState == "init": #state update comes after function call
  while len(scan_result) == 0: #state update comes after function call
    scan_result = pris_scan.scan(gQuit, grobotList, vWorld)
   
    time.sleep(.01)
  print 'scan result', scan_result
  
  #exit

  
def foo_to_baz():
  print "transition foo -> baz" 

def bar_to_foo():
  print "transition bar -> foo"

def baz_to_bar():
  print "transition baz -> bar"

def baz_to_fee():
  print "transition baz -> fee"

def fee_to_baz():
  print "transition fee -> baz"

def fee_to_foo():
  print "transition fee -> foo"

#thread to monitor events
def monitor_events():
  #global event_queue
  events = ["init", "scan", "baz", "fee"]
  
  while not gQuit:
    #when the correct number of hamsters are connect start program
    print "length grobotList", len(grobotList)
    if len(grobotList) == gMaxRobotNum:
      event_queue.put(Event("scan"))
    #event = Event(random.choice(events))
    #event_queue.put(event)
    time.sleep(1)

def build_states(pris_fsm):
  state_init = pris_fsm.add_state("init")
  state_scan = pris_fsm.add_state("scan")
  #state_baz = fsm.add_state("baz")
  #state_fee = fsm.add_state("fee")

  state_init.add_transition("scan", init_to_scan) #(next state name, function)
  #state_foo.add_transition("baz", foo_to_baz)
  #
  #state_bar.add_transition("foo", bar_to_foo)
  #
  #state_baz.add_transition("bar", baz_to_bar)
  #state_baz.add_transition("fee", baz_to_fee)
  #
  #state_fee.add_transition("baz", fee_to_baz)
  #state_fee.add_transition("foo", fee_to_foo)
    
def main():

  global event_queue
  global m
  global gQuit
  global grobotList
  global pris_fsm
  global gMaxRobotNum
  print "initializing global variables"
  gQuit = False
  gMaxRobotNum = 2; # max number of robots to control
  event_queue = None
  m = None
  grobotList = []
  event_queue = Queue.Queue()

    
  comm = RobotComm(gMaxRobotNum,-70) #maxRobot, minRSSI
  comm.start()
  print 'Bluetooth starts'
  #instanciate Robot
  grobotList = comm.get_robotList() 
  m = tk.Tk() #root
  drawQueue = Queue.Queue(0)
  motionQueue = Queue.Queue()

  #creating the virtual appearance of the robot
  canvas_width = 700 # half width
  canvas_height = 300  # half height
  rCanvas = tk.Canvas(m, bg="white", width=canvas_width*2, height=canvas_height*2)
  
  motionpath = pris_mp.MotionPath(comm, m, rCanvas, motionQueue)
  
  # visual elements of the virtual robot 
  poly_points = [0,0,0,0,0,0,0,0]
  motionpath.vrobot.poly_id = rCanvas.create_polygon(poly_points, fill='blue') #robot
  motionpath.vrobot.prox_l_id = rCanvas.create_line(0,0,0,0, fill="red") #prox sensors
  motionpath.vrobot.prox_r_id = rCanvas.create_line(0,0,0,0, fill="red")
  motionpath.vrobot.floor_l_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white") #floor sensors
  motionpath.vrobot.floor_r_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white")

  time.sleep(1)

  update_vrobot_thread = threading.Thread(target=motionpath.update_virtual_robot)
  update_vrobot_thread.daemon = True
  update_vrobot_thread.start()
  
  #create the virtual worlds that contains the virtual robot
  vWorld = pris_mp.virtual_world(drawQueue, motionpath.vrobot, rCanvas, canvas_width, canvas_height)
  
  #create motion path with x,y coordinates
  waypoint1 = [0,20]

  
  #add motion path to GUI
  vWorld.add_waypoint(waypoint1)


  #add motion path to Quene
  motionQueue.put(waypoint1)

  
  #start a thread to navigate the set path
  waypoint_thread = threading.Thread(target=motionpath.move_to_waypoint)
  waypoint_thread.daemon = True
  waypoint_thread.start()

  update_vrobot_thread = threading.Thread(target=motionpath.update_virtual_robot)
  update_vrobot_thread.daemon = True
  update_vrobot_thread.start()
  
  #create the virtual worlds that contains the virtual robot
  vWorld = gui.virtual_world(drawQueue, motionpath.vrobot, rCanvas, canvas_width, canvas_height)
  
  #objects in the world
  rect_l1 = [-150, 0, -110, 100]  #left 1
  rect_l2 = [-150, -100, -110, 0]  #left 2
  rect_b1 = [-150, -140, -50, -100]  #back 1
  rect_b2 = [-50, -140, 50, -100]  #back 2
  rect_b3 = [50, -140, 150, -100]    #back 3
  rect_b4 = [150, -140, 250, -100] #back 4
  rect_r1 = [210, 0, 250, 100]    #right 1
  rect_f1 = [-110, 98, -40, 100]    #front 1
  rect_f2 = [40, 98, 250, 100]    #front 2
  vWorld.add_obstacle(rect_l1)
  vWorld.add_obstacle(rect_l2)
  vWorld.add_obstacle(rect_b1)
  vWorld.add_obstacle(rect_b2)
  vWorld.add_obstacle(rect_b3)
  vWorld.add_obstacle(rect_b4)
  vWorld.add_obstacle(rect_r1)
  vWorld.add_obstacle(rect_f1)
  vWorld.add_obstacle(rect_f2)
  
  draw_world_thread = threading.Thread(target=pris_mp.draw_virtual_world, args=(vWorld, motionpath))
  draw_world_thread.daemon = True
  draw_world_thread.start()

  graphics = pris_mp.VirtualWorldGui(vWorld, m)
  

  pris_fsm = EventFsm()
  print "created a state machine"
  build_states(pris_fsm)

  pris_fsm.set_start("init")
  pris_fsm.set_current("init")

  event_thread = threading.Thread(target=monitor_events)
  event_thread.start()

  pris_fsm_thread = threading.Thread(target=pris_fsm.run)
  pris_fsm_thread.start()

  rCanvas.after(200, graphics.updateCanvas, drawQueue)
  m.mainloop()

  for robot in motionpath.gRobotList:
      robot.reset()
  comm.stop()
  comm.join()

  event_thread.join()


if __name__ == "__main__":
    main()

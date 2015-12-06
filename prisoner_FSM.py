'''
/* =======================================================================
Description:

    This file contains a finite state machine used by the prisoner robot.
   ========================================================================*/
'''

import final_config as gVars
import final_draw as draw
import final_GUI as gui
from HamsterAPI.comm_ble import RobotComm
import prisoner_scan as pris_scan   #import prisoner_bot as pris_bot
import Queue
import threading
import time
import Tkinter as tk 
import random

#set event names
class Event:
  def __init__(self, name):
    self.name = name

#define states from the FSM
class State:
    def __init__(self):
      self.name = ''
      self.transitions = {}

    def add_transition(self, toState, callback):
      self.transitions[toState] = callback
      
    def add_callback_args(self, args): #args in tuple format
      self.callback_args = args
      
    #store variables from each state, return_vars should be list format 
    def add_return_vars(self, return_vars):
      self.return_vars = return_vars 
 
#define componenets of FSM and run 
class EventFsm:
    def __init__(self):
        self.states = {}
        self.startState = None
        self.currentState = None
        self.pris_event_queue = Queue.Queue()

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
      while not gVars.gQuit:
        if self.pris_event_queue:
          event = self.pris_event_queue.get()
          print 'event', event.name
          if event.name in self.states[self.currentState].transitions:
            self.states[self.currentState].transitions[event.name](*self.states[self.currentState].callback_args)
            self.set_current(event.name)
        time.sleep(.1)

#--------------------------------------------------------------
#Callback functions

#begin scanning with the PSD sensor
def init_to_scan(vWorld, pris_fsm, prisoner):
  print "transition init -> scan"
  scan_result = [10, 20, 50, 60]   
  vWorld.add_decoy(scan_result)
  vWorld.draw_decoy()
  
  time.sleep(2)
  scan_result=[100, 80, 140, 120] 
  vWorld.add_decoy(scan_result)
  vWorld.draw_decoy()
    
  #start scanning
  scan_result = []
  while len(scan_result) == 0: #state update comes after function call
    scan_result = pris_scan.scan(vWorld)
    time.sleep(.01)
  
  time.sleep(1)
  pris_fsm.states["scan"].add_return_vars(scan_result)
  print 'scan result', pris_fsm.states["scan"].return_vars[0]
    
  #transition to next state
  if not (pris_fsm.states["scan"].return_vars[0] == 'no_decoy'):
    pris_fsm.pris_event_queue.put(Event("path"))
  else:
    pris_fsm.pris_event_queue.put(Event("end"))

#calculate and navigate motionpath
def scan_to_path(vWorld, pris_fsm, prisoner):
  print "transition scan -> path"
  
  #calculate motionpath based on box position
  prisoner.get_motionpath(vWorld, pris_fsm.states["scan"].return_vars)
  
  #ADD CODE TO CHECK WHERE THE GUARD IS BEFORE MOVING
  #start a thread to navigate the set path
  waypoint_thread = threading.Thread(target=prisoner.move_to_waypoint)
  waypoint_thread.daemon = True
  waypoint_thread.start()

  
#push box to 
def path_to_push(vWorld, pris_fsm, prisoner):
  print "transition path -> push"
  prisoner.push_decoy(vWorld)
  
    
def scan_to_end(vWorld,pris_fsm, prisoner):
  print "transition scan -> end"
  
  #end the program
  gVars.m.destroy()
  gVars.gQuit = True
  print "Exit"

#--------------------------------------------------------------
#dispatcher thread

#thread to monitor events
def monitor_events(pris_fsm, prisoner):
  start = True
  motionpath_done = False
  while not gVars.gQuit:
    #when the correct number of hamsters are connect start program
    if start == True and len(gVars.grobotList) == gVars.gMaxRobotNum:
      pris_fsm.pris_event_queue.put(Event("scan"))
      start = False
      
    #check to see if motionpath has been compeleted
    if prisoner.motion_done == True:      
      print"adding push to event queue because motionpath done"
      pris_fsm.pris_event_queue.put(Event("push"))
      prisoner.motion_done = False

    time.sleep(.01)
    
#--------------------------------------------------------------
#state building function 

def build_states(pris_fsm, vWorld, prisoner):
  #initialize state
  state_init = pris_fsm.add_state("init")
  state_init.add_transition("scan", init_to_scan) #(next state name, function)
  state_init.add_callback_args((vWorld,pris_fsm, prisoner)) #define arguements used in the transition callback funcs
  
  #scan for decoy box state
  state_scan = pris_fsm.add_state("scan")
  state_scan.add_transition("path", scan_to_path) 
  state_scan.add_transition("end", scan_to_end)
  state_scan.add_callback_args((vWorld, pris_fsm, prisoner))
  state_scan.add_return_vars([])  #define return variables for the state
  
  #navigate motionpath state
  state_path = pris_fsm.add_state("path")
  state_path.add_transition("push", path_to_push)
  state_path.add_callback_args((vWorld, pris_fsm, prisoner))
  state_path.add_return_vars([])
  
  #push the decoy box into position state
  state_push = pris_fsm.add_state("push")
  state_push.add_callback_args((vWorld, pris_fsm))
  state_push.add_return_vars([])
  
  #end state
  state_end = pris_fsm.add_state("end")

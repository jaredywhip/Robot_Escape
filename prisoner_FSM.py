import time
import threading
import Tkinter as tk 
import Queue
import random
from HamsterAPI.comm_ble import RobotComm
import final_draw as draw
#import prisoner_bot as pris_bot
import prisoner_scan as pris_scan
import final_GUI as gui
#from final_settings import *#file that contains global variables
import final_config as gVars


class Event:
  def __init__(self, name):
    self.name = name


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
          print "callback args" ,self.states[self.currentState].callback_args
          if event.name in self.states[self.currentState].transitions:
            self.states[self.currentState].transitions[event.name](*self.states[self.currentState].callback_args)
            self.set_current(event.name)
        time.sleep(.1)

def init_to_scan(vWorld, pris_fsm):
  #print pris_fsm.currentState
  print pris_fsm.states["scan"].transitions
  print "transition init -> scan"
   
  
  #start scanning
  scan_result = []
  #while pris_fsm.currentState == "init": #state update comes after function call
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
    

  
def scan_to_path(vWorld, pris_fsm, motionpath):
  print "transition scan -> path"
  #calculate motionpath based on box position
  motionpath.get_motionpath(vWorld, pris_fsm.states["scan"].return_vars)
  
  #start a thread to navigate the set path
  waypoint_thread = threading.Thread(target=motionpath.move_to_waypoint)
  waypoint_thread.daemon = True
  waypoint_thread.start()

def scan_to_end(vWorld,pris_fsm):
  print "transition scan -> end"
  gVars.gQuit = True

def baz_to_bar():
  print "transition baz -> bar"

def baz_to_fee():
  print "transition baz -> fee"

def fee_to_baz():
  print "transition fee -> baz"

def fee_to_foo():
  print "transition fee -> foo"

#thread to monitor events
def monitor_events(pris_fsm):
  start = True
  while not gVars.gQuit:
    #when the correct number of hamsters are connect start program
    if start == True and len(gVars.grobotList) == gVars.gMaxRobotNum:
      pris_fsm.pris_event_queue.put(Event("scan"))
      start = False
      
      
    ##transition to next state
    #if pris_fsm.states["scan"].return_vars:
    #  if not (pris_fsm.states["scan"].return_vars[0] == 'no_decoy'):
    #    event_queue.put(Event("push"))
    #  else:
    #    event_queue.put(Event("end"))

    time.sleep(.01)

def build_states(pris_fsm, vWorld, motionpath):
  state_init = pris_fsm.add_state("init")
  state_scan = pris_fsm.add_state("scan")
  state_path = pris_fsm.add_state("path")
  state_push = pris_fsm.add_state("push")
  state_end = pris_fsm.add_state("end")
  #state_baz = fsm.add_state("baz")
  #state_fee = fsm.add_state("fee")

  state_init.add_transition("scan", init_to_scan) #(next state name, function)'
  state_init.add_callback_args((vWorld,pris_fsm))
  
  state_scan.add_transition("path", scan_to_path) #(next state name, function)
  state_scan.add_transition("end", scan_to_end)
  state_scan.add_callback_args((vWorld, pris_fsm, motionpath))
  state_scan.add_return_vars([])
  
  state_path.add_callback_args((vWorld, pris_fsm, motionpath))
  state_path.add_return_vars([])
  
  state_push.add_callback_args((vWorld, pris_fsm))
  state_push.add_return_vars([])


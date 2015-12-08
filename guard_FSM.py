'''
/* =======================================================================
Description:

    This file contains a finite state machine used by the guard robot.
   ========================================================================*/
'''

import final_config as gVars
import final_draw as draw
import final_GUI as gui
from HamsterAPI.comm_ble import RobotComm
import guard_behaviors
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
        self.guard_event_queue = Queue.Queue()

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
        if self.guard_event_queue:
          event = self.guard_event_queue.get()
          if event.name in self.states[self.currentState].transitions:
            self.states[self.currentState].transitions[event.name](*self.states[self.currentState].callback_args)
            self.set_current(event.name)
        time.sleep(.1)

#--------------------------------------------------------------
#Callback functions

def init_to_scan(vWorld, guard_fsm, guard):
  prisoner_detected = guard_behaviors.scan(guard)
  print "guard current state", guard_fsm.currentState
  if (prisoner_detected):
    guard_fsm.guard_event_queue.put(Event("linetrace"))
  else:
    guard_fsm.guard_event_queue.put(Event("trap"))
  
def scan_to_linetrace(vWorld, guard_fsm, guard):
  guard_behaviors.linetrace(guard)
  guard_fsm.guard_event_queue.put(Event("scan"))

def linetrace_to_scan(vWorld, guard_fsm, guard):
  prisoner_detected = guard_behaviors.scan(guard)
  if (prisoner_detected):
    guard_fsm.guard_event_queue.put(Event("linetrace"))
  else:
    guard_fsm.guard_event_queue.put(Event("trap"))

def scan_to_trap(vWorld, guard_fsm, guard):
  print "Guard: 'The prisoner has tried to escape!'\n"
  alarm_thread = threading.Thread(target=sound_alarm,  args=(guard, guard_fsm,))
  alarm_thread.daemon = True
  alarm_thread.start()
  guard_behaviors.trap(guard)
  guard_fsm.guard_event_queue.put(Event("done"))

def trap_to_done(guard):
  guard_behaviors.done(guard)
  print "Press exit to end the program."
  #end the program
  

#--------------------------------------------------------------
# alarm thread

def sound_alarm(guard, guard_fsm):
  tick = 0
  while ((not gVars.gQuit) and (guard_fsm.currentState is not "done")):
    if (tick == 0):
      led_left = 1
      led_right = 4
      tick = 1
    else:
      led_left = 4
      led_left = 1
      tick = 0
    guard.robot.set_musical_note(80)
    guard.robot.set_led(0,led_left)
    guard.robot.set_led(1,led_right)
    time.sleep(.7)
    guard.robot.set_led(0,0)
    guard.robot.set_led(1,0)
    time.sleep(.05)
    guard.robot.set_musical_note(20)
    guard.robot.set_led(0,led_left)
    guard.robot.set_led(1,led_right)
    time.sleep(.7)
    guard.robot.set_led(0,0)
    guard.robot.set_led(1,0)
    time.sleep(.05)
  guard.robot.set_musical_note(0)
  guard.robot.set_led(0,0)
  guard.robot.set_led(1,0)


#--------------------------------------------------------------
# dispatcher thread

# thread to monitor events
def monitor_events(guard_fsm):
  start = True
  while not gVars.gQuit:
    #when the correct number of hamsters are connect start program
    if start == True and len(gVars.grobotList) == gVars.gMaxRobotNum:
      time.sleep(.5)
      guard_fsm.guard_event_queue.put(Event("scan"))
      start = False
    time.sleep(.01)
    
#--------------------------------------------------------------
# state building function 

def build_states(guard_fsm, vWorld, guard):
  # initialize state
  state_init = guard_fsm.add_state("init")
  state_init.add_transition("scan", init_to_scan) #(next state name, function)
  state_init.add_callback_args((vWorld, guard_fsm, guard)) #define arguements used in the transition callback funcs

  # scan state
  # if prisoner detected => linetrace state
  # if prisoner not detected => trap state
  state_scan = guard_fsm.add_state("scan")
  state_scan.add_transition("linetrace", scan_to_linetrace)
  state_scan.add_transition("trap", scan_to_trap)
  state_scan.add_callback_args((vWorld, guard_fsm, guard))

  # linetrace state
  state_linetrace = guard_fsm.add_state("linetrace")
  state_linetrace.add_transition("scan", linetrace_to_scan)
  state_linetrace.add_callback_args((vWorld, guard_fsm, guard))

  # trap state
  state_trap = guard_fsm.add_state("trap")
  state_trap.add_transition("done", trap_to_done)
  state_trap.add_callback_args((guard,))

  # done state
  state_done = guard_fsm.add_state("done")


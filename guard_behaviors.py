'''
/* =======================================================================
Description:

    This file contains functions used in the linetracing state of the guard.
   ========================================================================*/
'''

from collections import deque
import final_config as gVars
import math
import time
import guard_bot
import random
import Queue
import threading

prox_threshold = 27
pi = 3.14159
wheel_balance = -4

# returns True if the prisoner (or the decoy) is detected by the guard's scan, False otherwise
def scan(guard): 
    guard.robot.set_wheel_balance(wheel_balance)

    time.sleep(2)
    print "Guard: Checking to see if prisoner present.\n"
    prox_values = []
    for i in range(1,20):
        prox_values.append(guard.get_prox(0))
        prox_values.append(guard.get_prox(1))
        guard.move_left_slow()
        time.sleep(.03)
    guard.stop_move()
    for i in range(1,40):
        prox_values.append(guard.get_prox(0))
        prox_values.append(guard.get_prox(1))
        guard.move_right_slow()
        time.sleep(.03)
    guard.stop_move()
    for i in range(1,20):
        guard.move_left_slow()
        time.sleep(.03)
    guard.stop_move()
    count_above_threshold = 0
    for prox in prox_values:
        if (prox > prox_threshold):
            count_above_threshold = count_above_threshold + 1
    return (count_above_threshold > 3)

#function to close hallway blockade if escape detected
def trap(guard):
    print "Guard: Attempting to trap escaping prisoner.\n"
    guard.robot.set_wheel_balance(wheel_balance)
    guard.set_linetracer_mode_speed(0,0)
    guard.move_to_angle_right(3*pi/2)

    # correct for even proximities
    guard.rotate_till_even_proximities()
    guard.vrobot.a = 3*pi/2
    guard.vrobot.x = 0
    guard.vrobot.y = 140

    #scoot forward a bit to not detect current line
    guard.move_up()
    time.sleep(2)

    # push forward until black line passed
    lfloor = guard.get_floor(0)
    rfloor = guard.get_floor(1)
    while (lfloor > 60 and rfloor > 60):
        guard.move_up()
        time.sleep(.05)
        lfloor = guard.get_floor(0)
        rfloor = guard.get_floor(1)
    guard.stop_move()

#function to follow black tape line
def linetrace(guard, vWorld):
    # assumes guard starts off facing south ish after scanning 
    guard.alert_found_pris()
    
    print "Guard: Going to check on other prisoners.\n"

    #turn 90 degrees to the left to center on the line and check on other prisoners
    lfloor_list = []
    guard.move_left_slow()
    
    #populate list
    for i in range(0,11):
        lfloor_list.append(guard.get_floor(0))
    floor_thresh = 30
    while not all(j < floor_thresh for j in lfloor_list[-10:]):
        lfloor_list.append(guard.get_floor(0))
    
    #rotate past line to center onto line
    while not all(j > (floor_thresh + 55) for j in lfloor_list[-10:]):
        lfloor_list.append(guard.get_floor(0))
    guard.stop_move()

    #linetrace mode on, go to T
    guard.set_linetracer_mode_speed(6, 4)
    
    # wait for arbitrary amount of time while 'checking other cells'?
    wait_time = random.randint(7,20)
    print "Guard: Watching other prisoners for", wait_time, "seconds.\n"
    wait_time_str = str(wait_time)
    vWorld.add_countdown([400, 250, wait_time_str]) #timer = [xcoord, ycoord, 'time']
    vWorld.draw_countdown()
    
    #create a countdown timer in GUI
    countdown_timer = time.time() + wait_time
    while time.time() < countdown_timer:
        timer = countdown_timer - time.time()
        timer_str = str(int(timer))
        vWorld.add_countdown([400, 250, timer_str]) #timer = [xcoord, ycoord, 'time']
        vWorld.draw_countdown()
        time.sleep(1)
    
    #set GUI value to zero
    vWorld.add_countdown([400, 250, '0']) #timer = [xcoord, ycoord, 'time']
    vWorld.draw_countdown()
    
    #line trace mode off
    guard.set_linetracer_mode_speed(0,0)
    
    # localize along x axis 
    guard.localize()

    # turn 90 degrees to the right (until even proximities)
    guard.move_to_angle_right(pi)
    guard.rotate_till_even_proximities()
    guard.vrobot.a = pi

    # turn 90 degrees to the right to realign with line
    rfloor_list = []
    guard.move_right_slow()
    
    #populate list
    for i in range(0,11):
        rfloor_list.append(guard.get_floor(1))
    floor_thresh = 30
    while not all(j < floor_thresh for j in rfloor_list[-10:]):
        rfloor_list.append(guard.get_floor(1))
    
    #rotate past line to center onto line
    while not all(j > (floor_thresh + 55) for j in rfloor_list[-10:]):
        rfloor_list.append(guard.get_floor(1))
    guard.stop_move()
        
    # line trace mode on to go back to scan prisoner
    guard.set_linetracer_mode_speed(6, 4)
    time.sleep(7)
    # line trace mode off
    guard.set_linetracer_mode_speed(0,0)

    # correct until even proximies
    guard.rotate_till_even_proximities()
    guard.vrobot.a = 3*pi/2

    # localize along x axis
    guard.localize()

    # rotate left 90 degrees to face toward prisoner
    guard.move_to_angle_left(pi)
    
def done(guard):
    guard.robot.set_led(0,0)
    guard.robot.set_led(1,0)
    guard.robot.set_musical_note(0)
    print "Hallway secured. \n"

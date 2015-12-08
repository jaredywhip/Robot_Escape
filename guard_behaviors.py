'''
/* =======================================================================
Description:

    This file contains functions used in the line tracing state of the guard.
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
    print "guard is commencing scan"
    prox_values = []
    for i in range(1,20):
        prox_values.append(guard.get_prox(0))
        prox_values.append(guard.get_prox(1))
        guard.move_left_slow()
        time.sleep(.05)
    guard.stop_move()
    for i in range(1,40):
        prox_values.append(guard.get_prox(0))
        prox_values.append(guard.get_prox(1))
        guard.move_right_slow()
        time.sleep(.05)
    guard.stop_move()
    for i in range(1,20):
        guard.move_left_slow()
        time.sleep(.05)
    guard.stop_move()
    count_above_threshold = 0
    for prox in prox_values:
        if (prox > prox_threshold):
            count_above_threshold = count_above_threshold + 1
    return (count_above_threshold > 3)


def trap(guard):
    print "guard is attempting to trap escaping prisoner"
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



def linetrace(guard):
    # assumes guard starts off facing south ish after scanning 
    guard.alert_found_pris()
    
    print "Guard: Going to check on other prisoners.\n"
    
    # turn 90 degrees to the left 
    guard.move_to_angle_left(pi/2)

    #linetrace mode on
    guard.set_linetracer_mode_speed(6, 2)

    # wait for arbitrary amount of time while 'checking other cells'?
    wait_time = random.randint(20,40)
    print "Guard: Watching other prisoners for ", wait_time, "seconds.\n"
    time.sleep(wait_time)

    #line trace mode off
    guard.set_linetracer_mode_speed(0,0)

    # correct until even proximites
    guard.rotate_till_even_proximities()
    guard.vrobot.a = pi/2

    # localize along x axis 
    guard.localize()

    # turn 90 degrees to the right (until even proximities)
    guard.move_to_angle_right(pi)
    guard.rotate_till_even_proximities()
    guard.vrobot.a = pi

    # turn 90 degrees to the right
    guard.move_to_angle_right(3*pi/2)

    # line trace mode on
    guard.set_linetracer_mode_speed(6, 2)
    time.sleep(6)
    # line trace mode off
    guard.set_linetracer_mode_speed(0,0)

    # correct until even proximies
    guard.rotate_till_even_proximities()
    guard.vrobot.a = 3*pi/2

    # localize along x axis
    guard.localize()

    # rotate left 90 degrees to face south
    guard.move_to_angle_left(pi)

def done(guard):
    guard.robot.set_led(0,0)
    guard.robot.set_led(1,0)
    guard.robot.set_musical_note(0)
    print "Done"

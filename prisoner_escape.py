'''
/* =======================================================================
Description:

    This file contains functions used in the scanning state of the prisoner.
   ========================================================================*/
'''

from collections import deque
import final_config as gVars
import math
import time

#define common angles
pi4 = 3.14159/4
pi2 = 3.14159/2
p3i4 = (3*3.14159)/4
pi = 3.14159
p5i4 = (5*3.14159)/4
p3i2 = (3*3.14159)/2
p7i4 = (7*3.14159)/4
p2i = 2 * 3.14159

def move_to_angle_left(prisoner, rot_ang):
    #turn robot to angle
    current_a = prisoner.vrobot.a        
    curr_delt_a = rot_ang - current_a
    prisoner.move_left()  
    while abs(curr_delt_a) > .03:
        curr_delt_a = rot_ang - current_a
        current_a = prisoner.vrobot.a
        time.sleep(.01)
    prisoner.stop_move()

def move_to_angle_right(prisoner, rot_ang):
    #turn robot to angle
    current_a = prisoner.vrobot.a        
    curr_delt_a = rot_ang - current_a
    prisoner.move_right()  
    while abs(curr_delt_a) > .03:
        curr_delt_a = rot_ang - current_a
        current_a = prisoner.vrobot.a
        time.sleep(.01)
    prisoner.stop_move()

def escape(vWorld,pris_fsm, prisoner):
    
    print "in escape file, pris x", prisoner.vrobot.x
    robot = prisoner.robot
    
    #back up robot
    backup_timer = time.time() + 2
    prisoner.move_down()
    while time.time() < backup_timer:
        time.sleep(.01)
    prisoner.stop_move()

    #turn robot to align with back wall
    move_to_angle_left(prisoner,pi)
    
    time.sleep(.3)
    
    vfinal_x = prisoner.vrobot.x
    vfinal_y = -100 + 30
    vfinal_a = pi
    
    #drive to back wall
    prisoner.dist_move(['dist_move', 30, vfinal_x , vfinal_y, vfinal_a])
    
    #turn to hole in the left wall
    move_to_angle_right(prisoner,p3i2)
    time.sleep(.3)
    
    # make sure the robot doesn't go out of bounds
    floor_thresh = 50
    
    prisoner.move_up()
    while not prisoner.check_line(floor_thresh):
        time.sleep(0.001)
    prisoner.stop_move()
    
    #localize
    prisoner.vrobot.x = -190
    prisoner.vrobot.y = -69
    prisoner.vrobot.a = p3i2
    
    
    #turn robot to align toward hallway exit
    move_to_angle_right(prisoner,0)
    
    time.sleep(.3)
    
    #drive the robot to final target
    prisoner.move_up()
    prox_thresh = 60
    while not prisoner.check_line(floor_thresh):
        prisoner.correct_right(prox_thresh,'move_up')
        prisoner.correct_left(prox_thresh,'move_up')
        time.sleep(0.001)
    prisoner.stop_move()
    
    print "escaped!"

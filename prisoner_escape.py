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

#--------------------------------------------------------------
#helper functions

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

#--------------------------------------------------------------
#prisoner escape function

def escape(vWorld,pris_fsm, prisoner, guard_fsm):
    robot = prisoner.robot
    
    #back up robot for 2 seconds
    backup_timer = time.time() + 2
    prisoner.move_down()
    while time.time() < backup_timer:
        time.sleep(.01)
    prisoner.stop_move()

    #turn robot to align with back wall
    move_to_angle_left(prisoner,pi)
    
    time.sleep(.3) #pause
    
    #set variables for distance move
    vfinal_x = prisoner.vrobot.x
    vfinal_y = -100 + 35
    vfinal_a = pi
    
    if guard_fsm.currentState == 'trap' or guard_fsm.currentState == 'done':
        return
            
    #drive to back wall
    prisoner.dist_move(['dist_move', 35, vfinal_x , vfinal_y, vfinal_a])
    
    if guard_fsm.currentState == 'trap' or guard_fsm.currentState == 'done':
        return
    
    #turn to hole in the left wall
    move_to_angle_right(prisoner,(p3i2))
    
    #make sure robot perpendicular to left wall
    tol = 2
    prox_l = prisoner.robot.get_proximity(0)
    prox_r = prisoner.robot.get_proximity(1)
    if not prox_r - tol < prox_l < prox_r + tol:
        prisoner.move_right_slow()
        while not prox_r - tol < prox_l < prox_r + tol:
            prox_l = prisoner.robot.get_proximity(0)
            prox_r = prisoner.robot.get_proximity(1)
            #check to see if escape was discovered
            if guard_fsm.currentState == 'trap' or guard_fsm.currentState == 'done':
                return
                break            
            time.sleep(.001)
            
    time.sleep(.3)
    
    #set threshold for floor sensors
    floor_thresh = 50
    
    #drive out of cell and into hallway
    prisoner.move_up_fast()
    
    while not prisoner.check_line(floor_thresh):
        if guard_fsm.currentState == 'trap' or guard_fsm.currentState == 'done':
            return
            break
        time.sleep(0.001)
    prisoner.stop_move()
    
    #localize in hallway
    prisoner.vrobot.x = -200
    prisoner.vrobot.y = -69
    prisoner.vrobot.a = p3i2
    
    #turn robot to align toward hallway exit
    move_to_angle_right(prisoner,0)
    
    time.sleep(.3) #pause
    
    #drive the robot to final target
    prisoner.move_up_fast()
    prox_thresh = 50
    while not prisoner.check_line(floor_thresh):
        prisoner.correct_right(prox_thresh,'move_up')
        prisoner.correct_left(prox_thresh,'move_up')
        if guard_fsm.currentState == 'trap' or guard_fsm.currentState == 'done':
            return
            break
        time.sleep(0.001)
    prisoner.stop_move()
    
    #localize at final target
    prisoner.vrobot.x = -205
    prisoner.vrobot.y = 210
    prisoner.vrobot.a = 0
    
    print "Prisoner: Haha! I fooled the gaurd and escaped! \n"
    prisoner.celebrate_song()
    

'''
/* =======================================================================
Description:

    This file contains the main function for the CS123 prisoner escape
    final project.
   ========================================================================*/
'''

import final_config as gVars
import final_draw as draw
import final_GUI as GUI
import guard_bot as guard_bot
import guard_FSM as guard_FSM
from HamsterAPI.comm_ble import RobotComm
import prisoner_bot as pris_bot
import prisoner_FSM as pris_FSM
import prisoner_scan as pris_scan
import Queue
import random
import threading
import time
import Tkinter as tk

def main():
      
    comm = RobotComm(gVars.gMaxRobotNum,-70) #maxRobot, minRSSI
    comm.start()
    print 'Bluetooth starts'
    #instanciate robot list and GUI root
    gVars.grobotList = comm.get_robotList()
    gVars.m = tk.Tk() #root
    drawQueue = Queue.Queue(0)
  
    #creating the GUI canvas
    canvas_width = 700 # half width
    canvas_height = 300  # half height
    rCanvas = tk.Canvas(gVars.m, bg="white", width=canvas_width*2, height=canvas_height*2)
    
    #create the prisoner vitual robot 
    prisoner = pris_bot.Prisoner(comm, rCanvas)
    
    # visual elements of the virtual robot 
    poly_points = [0,0,0,0,0,0,0,0]
    prisoner.vrobot.poly_id = rCanvas.create_polygon(poly_points, fill='orange') #robot
    prisoner.vrobot.prox_l_id = rCanvas.create_line(0,0,0,0, fill="red") #prox sensors
    prisoner.vrobot.prox_r_id = rCanvas.create_line(0,0,0,0, fill="red")
    prisoner.vrobot.floor_l_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white") #floor sensors
    prisoner.vrobot.floor_r_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white")
    
    update_pvrobot_thread = threading.Thread(target=prisoner.update_virtual_robot)
    update_pvrobot_thread.daemon = True
    update_pvrobot_thread.start()
    
    
    #create the prisoner vitual robot 
    guard = guard_bot.Guard(comm, rCanvas)
    
    # visual elements of the virtual robot 
    poly_points = [0,0,0,0,0,0,0,0]
    guard.vrobot.poly_id = rCanvas.create_polygon(poly_points, fill='green') #robot
    guard.vrobot.prox_l_id = rCanvas.create_line(0,0,0,0, fill="red") #prox sensors
    guard.vrobot.prox_r_id = rCanvas.create_line(0,0,0,0, fill="red")
    guard.vrobot.floor_l_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white") #floor sensors
    guard.vrobot.floor_r_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white")
  
    update_gvrobot_thread = threading.Thread(target=guard.update_virtual_robot)
    update_gvrobot_thread.daemon = True
    update_gvrobot_thread.start()
    
    #create the virtual worlds that contains the virtual robot
    vWorld = draw.virtual_world(drawQueue, prisoner.vrobot, guard.vrobot, rCanvas, canvas_width, canvas_height)
    
    #objects in the world
    rect_slider = [-150, 160, -60, 120]
    rect_l1 = [-150, 0, -110, 120]  #left 1
    rect_l2 = [-112, -100, -110, -5]  #left 2 (paper flap)
    rect_b1 = [-250, -105, 265, -100]  #back wall
    rect_r1 = [260, -100, 265, 200]    #right 1
    rect_f1 = [-110, 100, -40, 105]    #front 1
    rect_f2 = [40, 100, 260, 105]    #front 2
    rect_lh = [-250, -100, -245, 210] #left hallway wall
    rect_rh = [-150, 160, -145, 210]
    vWorld.add_obstacle(rect_l1)
    vWorld.add_obstacle(rect_l2)
    vWorld.add_obstacle(rect_b1)
    vWorld.add_obstacle(rect_r1)
    vWorld.add_obstacle(rect_f1)
    vWorld.add_obstacle(rect_f2)
    vWorld.add_obstacle(rect_lh)
    vWorld.add_obstacle(rect_rh)
    vWorld.add_obstacle(rect_slider)
    
    
    #create boundary for decoy box starting position
    vWorld.add_boundary([50,0,170,80])
    
    #start thread to draw the virtual world
    draw_world_thread = threading.Thread(target=GUI.draw_virtual_world, args=(vWorld, prisoner, guard))
    draw_world_thread.daemon = True
    draw_world_thread.start()
  
    graphics = GUI.VirtualWorldGui(vWorld, gVars.m)
    
    #create FSM for guard
    guard_fsm = guard_FSM.EventFsm()
    print "created the guard state machine"
    guard_FSM.build_states(guard_fsm, vWorld, guard)
    guard_fsm.set_start("init")
    guard_fsm.set_current("init")
    
    #start thread to monitor events in guard FSM
    guard_event_thread = threading.Thread(target=guard_FSM.monitor_events,  args=(guard_fsm,))
    guard_event_thread.daemon = True
    guard_event_thread.start()
    
    #start thread to run guard FSM
    guard_fsm_thread = threading.Thread(target=guard_fsm.run)
    guard_fsm_thread.daemon = True
    guard_fsm_thread.start()
    
    #create FSM for prisoner
    pris_fsm = pris_FSM.EventFsm()
    print "created the prisoner state machine"
    pris_FSM.build_states(pris_fsm, vWorld, prisoner, guard_fsm)
    pris_fsm.set_start("init")
    pris_fsm.set_current("init")
  
    #start thread to monitor events in prisoner FSM
    pris_event_thread = threading.Thread(target=pris_FSM.monitor_events,  args =(pris_fsm, prisoner, guard_fsm))
    pris_event_thread.daemon = True
    pris_event_thread.start()
    
    #start thread to run prisoner FSM
    pris_fsm_thread = threading.Thread(target=pris_fsm.run)
    pris_fsm_thread.daemon = True
    pris_fsm_thread.start()
  
                
    rCanvas.after(200, graphics.updateCanvas, drawQueue)
    gVars.m.mainloop()
    
    print "Cleaning up"
    
    for robot in gVars.grobotList:
        robot.reset()
    comm.stop()
    comm.join()
    if pris_event_thread.isAlive():
        pris_event_thread.join(.5)
    if pris_fsm_thread.isAlive():    
        pris_fsm_thread.join(.5)
    if guard_event_thread.isAlive():
        guard_event_thread.join(.5)
    if guard_fsm_thread.isAlive():    
        guard_fsm_thread.join(.5) 
    if draw_world_thread.isAlive():
        draw_world_thread.join(.5)

    print "Goodbye."
      
if __name__ == "__main__":
    main()

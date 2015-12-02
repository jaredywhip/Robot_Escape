from collections import deque
import math
import time


#function to control servo
def write_servo_position(deg, robot):
    robot.set_io_mode(1, 0x08) #Servo control
    robot.set_port(1, deg)
    pass #move the servo motor to deg

#low pass filter; data must be a 2 element list
def low_pass(data, max_len):
    lp_data = data
    alpha = .15 #empirically set, small means smoother
    
    #lowpass filter eqn        
    lp_data[max_len - 1] = alpha * data[max_len - 1] + (1-alpha) * data[max_len - 2] #data[1] is new element, data[0] is old
    return lp_data

#function to read psd sensor
def read_psd_distance(robot):
    robot.set_io_mode(0, 0x00) #Analog to Digital(default value)
    psd_value = robot.get_port(0)
    return psd_value #read psd distance

def rotate_servo(deg_timer, dir, delta, deg, period, robot):
    #rotate servo
    if deg > 0 and (time.time() > deg_timer):
        #move to the desired position: {deg| 0 < deg <=180}
        deg = deg + (dir * delta)
        
        write_servo_position(deg,robot) #implement
        deg_timer = time.time() + period
    return (deg, deg_timer)

def calc_decoy_pos(leading_deg, dist_psd):
    #calc psd offset at leading deg
    #knowing robot center point at 0,20
    #servo psd sensor arm length is 22mm and servo rotational center at 0,12
    dist_corn = dist_psd + 22
    x_corn = math.cos(math.radians(leading_deg)) * dist_corn
    y_corn = math.sin(math.radians(leading_deg)) * dist_corn + 12
    
    box_x1 = x_corn
    box_y1 = y_corn - 40 
    box_x2 = x_corn + 40
    box_y2 = y_corn
    
    return (box_x1, box_y1, box_x2, box_y2)

def scan(gQuit, grobotList):
    print "Scanning for decoy!"
    
    
    #set max length of data that will be low pass filtered
    max_len = 5
    #this creates a list of two elements. it adds new element in pos [1] and pops elem in pos[0]
    psd_list = deque(maxlen = max_len)
    
    threshold = 140
    
    #instanciate variables
    dir = -1
    delta = 1
    deg = 180 #points left
    line_psd = {}
    
    for i in range(0, 181):
        line_psd[i] = None
        
    scanning_period = 6.0
    period = scanning_period/len(line_psd)*delta
    print "scanning period: 180deg:",period, "sec"
    
    #set timer for servo
    deg_timer = time.time() + period
    
    #define scan result list
    scan_result = []
    

    #scan
    while (not gQuit):
            robot = grobotList[0]
            
            #rotate servo
            deg = rotate_servo(deg_timer, dir, delta, deg, period, robot)[0]
            deg_timer = rotate_servo(deg_timer, dir, delta, deg, period, robot)[1]
            
            psd_value = read_psd_distance(robot) #implement
            mag = 255 - psd_value;
            psd_list.append(mag)
            
            #initialize list; add value if only one element
            if len(psd_list) < max_len:
                while len(psd_list) <= (max_len - 1):
                    psd_list.append(mag)
            
            #low pass filter the data, return deque
            lpf_mag = low_pass(psd_list, max_len)
            
            print deg, lpf_mag[max_len - 1]
            
            #detect leading edge of object
            if all(val <= threshold for val in lpf_mag):
                print 'OBJECT!'
                print lpf_mag
                leading_deg = deg
                time.sleep(1)
                
                #rotate to the center of the box
                if deg > 5:
                    write_servo_position(deg - 5,robot) #implement
                else:
                    write_servo_position(0,robot) #implement
                    
                time.sleep(1)
                #get stable sensor values
                for i in range(0,5):
                    psd_value = read_psd_distance(robot) #implement
                    mag = 255 - psd_value;
                    psd_list.append(mag)
                    #low pass filter the data, return deque
                    lpf_mag = low_pass(psd_list, max_len)
                
                print lpf_mag
                
                #calculate distance
                min_psd = min(psd_list)
                leading_deg = deg
                dist_psd = 1.0169 * min_psd - 7.4546 #empiracle data linear fit, confidence range 60 - 180mm
                
                decoy_coord = calc_decoy_pos(leading_deg, dist_psd)
                
                #center the psd sensor
                while deg <= 90:
                    dir = 1
                    deg = rotate_servo(deg_timer, dir, delta, deg, period, robot)[0]
                    deg_timer = rotate_servo(deg_timer, dir, delta, deg, period, robot)[1]
                
                scan_result = [decoy_coord[0], decoy_coord[1], decoy_coord[2], decoy_coord[3]]   
                break
                
            
            time.sleep(.001)
    
    print "scan complete"
    return scan_result
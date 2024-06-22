from __future__ import division
import time
import math as m
import numpy as np
import array as arr 
import timeit
import threading
from sshkeyboard import listen_keyboard


# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()
mpu = Adafruit_PCA9685.MPU6050()

# Configure min and max servo pulse lengths
servo_min = 100  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

l1=46.4 #mm
l2=114.2 #mm
l3=110 #mm

RH = -140 # mm -> Robot height
SH = 45 # mm -> Swing height

#Define side
Front = 30
Rear = 31

#Define leg adress
front_left = 0
front_right = 3
rear_left = 6
rear_right = 9

# Offset matrix [t1,t2,t3] <=> [0,45,90]
FL = arr.array('i', [92, 62, 12])   # -> <Front Left>
RL = arr.array('i', [85, 93, 24])   # -> <Rear Left>
FR = arr.array('i', [82, 83, 183])  # -> <Front Right>
RR = arr.array('i', [92, 95, 180])  # -> <Front Right>

# End legs position
posFL = np.array([0, 0, 0])   # -> <Front Left>
posRL = np.array([0, 0, 0])   # -> <Rear Left>
posFR = np.array([0, 0, 0])  # -> <Front Right>
posRR = np.array([0, 0, 0])  # -> <Front Right>

# angle to pulse
def angle2pulse(angle):
    return (int)(100+500*angle/180)

# Control leg
def setLegAngles(LegAdress, Theta1, Theta2, Theta3):
    #offset
    if(LegAdress == front_left):
        Theta1 = 180 - (Theta1 + FL[0])
        Theta2 = Theta2 + FL[1]
        Theta3 = Theta3 + FL[2]
    elif(LegAdress == front_right):
        Theta1 = 180 - (Theta1 + FR[0])
        Theta2 = Theta2 + FR[1]
        Theta3 = Theta3 + FR[2]
    elif(LegAdress == rear_left):
        Theta1 = Theta1 + RL[0]
        Theta2 = Theta2 + RL[1]
        Theta3 = Theta3 + RL[2]
    elif(LegAdress == rear_right):
        Theta1 = Theta1 + RR[0]
        Theta2 = Theta2 + RR[1]
        Theta3 = Theta3 + RR[2]
    else: 
        print("Wrong Leg Adress")
        exit()
    # print('Theta1= ',Theta1,'Theta2= ',Theta2,'Theta3= ',Theta3)
    pwm.set_pwm_servo(LegAdress,0,angle2pulse(Theta1),angle2pulse(Theta2),angle2pulse(Theta3))
    
# setup function
def Calib():
    setLegAngles(front_left, 0, 45, 90)
    setLegAngles(front_right, 0, -45, -90)
    setLegAngles(rear_left, 0, 45, 90)
    setLegAngles(rear_right, 0, -45, -90)
    exit()
    
# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

def Check_work_space(x,y,z):
    k1 = x**2 + y**2 + z**2
    k2 = y**2 + z**2
    if( k2 < (l1**2 + (l2 - l3)**2) or k1 > (l1**2 + (l2 + l3)**2)):
        print("The position :(",x,y,z,") is Out of workspace")
        exit()

def LEFT_Inverse_Kinematics(leg,x,y,z):
    global posFL, posRL, posFR, posRR
    #check work space
    Check_work_space(x,y,z)
    # Theta1
    alpha = m.acos(y/np.sqrt(y**2 + z**2))
    t1 = -(m.acos(l1/np.sqrt(z**2 + y**2)) - alpha)
    c1 = np.cos(t1)
    s1 = np.sin(t1)
    # Theta3
    A = -x
    if(t1!=0):
        B = (l1*c1 - y)/s1
    else:
        B = (-l1*s1 - z)/c1
    c3 = (A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3)
    s3 = np.sqrt(m.fabs(1 - c3**2))
    t3 = m.atan2(s3, c3)
    # Theta2
    s2 = (A * (l2 + l3 * c3) + B * l3 * s3)  
    c2 = (B * (l2 + l3 * c3) - A * l3 * s3)  
    t2 = m.atan2(s2, c2)

    # Convert Rad -> Deg
    t1 = t1*180/np.pi
    t2 = t2*180/np.pi
    t3 = t3*180/np.pi

    # Run and save new position
    if(leg == Rear):
        setLegAngles(rear_left,t1,t2,t3)
        posRL = [x, y, z] 
    elif(leg == Front):
        setLegAngles(front_left,t1,t2,t3)
        posFL = [x, y, z] 

def RIGHT_Inverse_Kinematics(leg,x,y,z):
    global posFL, posRL, posFR, posRR
    #check work space
    Check_work_space(x,y,z)
    # Theta1
    alpha = m.asin(y/np.sqrt(y**2 + z**2))
    t1 = -(m.asin(l1/np.sqrt(z**2 + y**2)) + alpha)
    c1 = np.cos(t1)
    s1 = np.sin(t1)
    # Theta3
    A = -x
    if(t1!=0):
        B = (-l1*c1 - y)/s1
    else:
        B = (l1*s1 - z)/c1
    c3 = (A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3)
    s3 = np.sqrt(m.fabs(1 - c3**2))
    t3 = -m.atan2(s3, c3)
    # Theta2
    s2 = (A * (l2 + l3 * c3) + B * l3 * s3)  
    c2 = (B * (l2 + l3 * c3) - A * l3 * s3)  
    t2 = -m.atan2(s2, c2)
    
    # Convert Rad -> Deg
    t1 = t1*180/np.pi
    t2 = t2*180/np.pi
    t3 = t3*180/np.pi
    
    # Run and save new position
    if(leg == Rear):
        setLegAngles(rear_right,t1,t2,t3)
        posRR = [x, y, z] 
    elif(leg == Front):
        setLegAngles(front_right,t1,t2,t3)
        posFR = [x, y, z] 
    
###
Roll = 0
Pitch = 0
# preTimeGet = 0
def getRollPitch_MPU():
    global Roll, Pitch # ,preTimeGet
    Roll, Pitch = mpu.get_RP_Angle()
    print("Goc truc X: %.1f" %Roll  ,  "Goc truc Y: %.1f" %Pitch)
    self_balancing_pitch(Pitch)
    self_balancing_roll(Roll)
    threading.Timer(0.02,getRollPitch_MPU,).start()
    # timeGet = timeit.default_timer()
    # print(timeGet - preTimeGet, " (seconds)")
    # preTimeGet = timeGet
###

'''
self balancing test 
'''
def PID(setpoint,current_value):
    kp=0.125 #0.15
    ki=0.01 #0.02
    kd=0.001 #0.002
    sample_time = 0.02
    err = setpoint - current_value # roll,pitch angle 
    err_p =0 
    iterm_p = 0

    #k term 
    kterm = err*kp

    #iterm
    iterm = iterm_p + ki*err*sample_time 

    #dterm 
    dterm = kd*(err-err_p)/sample_time

    #update data 
    iterm_p = iterm
    err_p = err

    pidterm  = kterm + iterm + dterm

    #saturation 
    if(pidterm < -20) : output = -20
    elif(pidterm >20) :output = 20
    else : output = pidterm
    # print(output)
    return output

def self_balancing_pitch(pitch_angle):
    # print("pitch \n")
    global posFL, posFR, posRR, posRL
    # x = 239.6
    x = 269.5
    #front leg
    posFL[2]  = posFL[2] + x/2*np.tan(PID(0,pitch_angle)*np.pi/180)
    posFR[2]  = posFR[2] + x/2*np.tan(PID(0,pitch_angle)*np.pi/180)

    RIGHT_Inverse_Kinematics(Front, 0,-l1,posFR[2])
    LEFT_Inverse_Kinematics(Front, 0,l1,posFL[2])
    #rear leg
    posRL[2] = posRL[2] - x/2*np.tan(PID(0,pitch_angle)*np.pi/180)
    posRR[2] = posRR[2] - x/2*np.tan(PID(0,pitch_angle)*np.pi/180)
    RIGHT_Inverse_Kinematics(Rear, 0,-l1,posRR[2])
    LEFT_Inverse_Kinematics(Rear, 0,l1,posRL[2])

def self_balancing_roll(roll_angle):
    # print("roll \n")
    global posFL, posFR, posRR, posRL
    x= 80.3 + 2*l1
    #right leg
    posFR[2] = posFR[2] + x/2*np.tan(PID(0,roll_angle)*np.pi/180)
    posRR[2] = posRR[2] + x/2*np.tan(PID(0,roll_angle)*np.pi/180)
    # print(new_height_right)
    RIGHT_Inverse_Kinematics(Front, 0,-l1,posFR[2])
    RIGHT_Inverse_Kinematics(Rear, 0,-l1,posRR[2])
    #left leg
    posFL[2] = posFL[2] - x/2*np.tan(PID(0,roll_angle)*np.pi/180)
    posRL[2] = posRL[2] - x/2*np.tan(PID(0,roll_angle)*np.pi/180)
    # print(new_height_left)
    LEFT_Inverse_Kinematics(Front, 0,l1,posFL[2])
    LEFT_Inverse_Kinematics(Rear, 0,l1,posRL[2])

'''
----------------------------------------------
'''

def initial_position():
    LEFT_Inverse_Kinematics(Front, 0,l1,RH)
    RIGHT_Inverse_Kinematics(Front, 0,-l1,RH)
    LEFT_Inverse_Kinematics(Rear, 0,l1,RH)
    RIGHT_Inverse_Kinematics(Rear, 0,-l1,RH) 
    time.sleep(1)
    print("Ready")

def all_Move(x, y, z, Time_delay): # di chuyen tat ca cac chan 1 doan theo huong x,y,z
    FR = posFR
    RR = posRR
    FL = posFL
    RL = posRL
    # for t in np.arange(0.125, 1.005, 0.125):
    for t in np.arange(0.25, 1.005, 0.25):
        RIGHT_Inverse_Kinematics(Front, FR[0] - x*t, FR[1] + y*t, FR[2] - z*t)
        RIGHT_Inverse_Kinematics(Rear, RR[0] - x*t, RR[1] + y*t, RR[2] - z*t)
        LEFT_Inverse_Kinematics(Rear, RL[0] - x*t, RL[1] + y*t, RL[2] - z*t)
        LEFT_Inverse_Kinematics(Front, FL[0] - x*t, FL[1] + y*t, FL[2] - z*t)
        time.sleep(Time_delay)
    
def all_To_initial(Time_delay):
    FR = posFR
    RR = posRR
    FL = posFL
    RL = posRL
    # for t in np.arange(0.125, 1.005, 0.125):
    for t in np.arange(0.25, 1.005, 0.25):
        RIGHT_Inverse_Kinematics(Front, FR[0] - FR[0]*t, FR[1] - (FR[1]+l1)*t, FR[2] - (FR[2]-RH)*t)
        RIGHT_Inverse_Kinematics(Rear, RR[0] - RR[0]*t, RR[1] - (RR[1]+l1)*t, RR[2] - (RR[2]-RH)*t)
        LEFT_Inverse_Kinematics(Rear, RL[0] - RL[0]*t, RL[1] - (RL[1]-l1)*t, RL[2] - (RL[2]-RH)*t)
        LEFT_Inverse_Kinematics(Front, FL[0] - FL[0]*t, FL[1] - (FL[1]-l1)*t, FL[2] - (FL[2]-RH)*t)
        time.sleep(Time_delay)


A = 269.5 # robot length
B = 80.3 # robot width
g_FL = np.array([[A/2],[B/2],[0]])
g_FR = np.array([[A/2],[-B/2],[0]])
g_RL = np.array([[-A/2],[B/2],[0]])
g_RR = np.array([[-A/2],[-B/2],[0]])

pre_Roll_RPY = 0
pre_Pitch_RPY = 0
pre_Yaw_RPY = 0
def Roll_Pitch_Yaw(roll, pitch, yaw):
    global pre_Roll_RPY, pre_Pitch_RPY, pre_Yaw_RPY

    Roll_RPY = roll*np.pi/180
    roll = Roll_RPY - pre_Roll_RPY
    pre_Roll_RPY = Roll_RPY
 
    Pitch_RPY = pitch*np.pi/180
    pitch = Pitch_RPY - pre_Pitch_RPY
    pre_Pitch_RPY = Pitch_RPY

    Yaw_RPY = yaw*np.pi/180
    yaw = Yaw_RPY - pre_Yaw_RPY
    pre_Yaw_RPY = Yaw_RPY  
    
    pos_FL = np.array([[posFL[0]],[posFL[1]],[posFL[2]]]) + g_FL
    pos_FR = np.array([[posFR[0]],[posFR[1]],[posFR[2]]]) + g_FR
    pos_RL = np.array([[posRL[0]],[posRL[1]],[posRL[2]]]) + g_RL
    pos_RR = np.array([[posRR[0]],[posRR[1]],[posRR[2]]]) + g_RR
    
    rotationX = np.array([  [1,     0,               0              ],
                            [0,     np.cos(-roll),   -np.sin(-roll) ],
                            [0,     np.sin(-roll),   np.cos(-roll)  ]])

    rotationY = np.array([  [np.cos(-pitch),    0,  np.sin(-pitch)  ],
                            [0,                 1,  0               ],
                            [-np.sin(-pitch),   0,  np.cos(-pitch)  ]])
    
    rotationZ = np.array([  [np.cos(-yaw),  -np.sin(-yaw),  0   ],
                            [np.sin(-yaw),  np.cos(-yaw),   0   ],
                            [0,             0,              1   ]])

    new_posFL = rotationX.dot(pos_FL)
    new_posFR = rotationX.dot(pos_FR) 
    new_posRL = rotationX.dot(pos_RL) 
    new_posRR = rotationX.dot(pos_RR) 
    
    new_posFL = rotationY.dot(new_posFL) 
    new_posFR = rotationY.dot(new_posFR)
    new_posRL = rotationY.dot(new_posRL) 
    new_posRR = rotationY.dot(new_posRR) 
    
    new_posFL = rotationZ.dot(new_posFL) - g_FL
    new_posFR = rotationZ.dot(new_posFR) - g_FR
    new_posRL = rotationZ.dot(new_posRL) - g_RL
    new_posRR = rotationZ.dot(new_posRR) - g_RR
    
    print(new_posFL[0,0],new_posFL[1,0],new_posFL[2,0])
    print(new_posFR[0,0],new_posFR[1,0],new_posFR[2,0])
    print(new_posRL[0,0],new_posRL[1,0],new_posRL[2,0])
    print(new_posRR[0,0],new_posRR[1,0],new_posRR[2,0])

    LEFT_Inverse_Kinematics(Front, round(new_posFL[0,0],2), round(new_posFL[1,0],2), round(new_posFL[2,0],2))
    RIGHT_Inverse_Kinematics(Front, round(new_posFR[0,0],2), round(new_posFR[1,0],2), round(new_posFR[2,0],2))
    LEFT_Inverse_Kinematics(Rear, round(new_posRL[0,0],2), round(new_posRL[1,0],2), round(new_posRL[2,0],2))
    RIGHT_Inverse_Kinematics(Rear, round(new_posRR[0,0],2), round(new_posRR[1,0],2), round(new_posRR[2,0],2))


pressed_key = None
### ReadKey to control Roll Pitch Yaw 
Roll_rpy= 0
Pitch_rpy= 0
Yaw_rpy= 0
def on_press_1(key):
    global Roll_rpy, Pitch_rpy, Yaw_rpy, pressed_key
    pressed_key = key
    while(pressed_key == '1'):
        Roll_rpy  += 3
        Roll_Pitch_Yaw(Roll_rpy,Pitch_rpy,Yaw_rpy)
        time.sleep(0.1)
    while(pressed_key == '2'):
        Roll_rpy  -= 3
        Roll_Pitch_Yaw(Roll_rpy,Pitch_rpy,Yaw_rpy)
        time.sleep(0.1)
    while(pressed_key == '3'):
        Pitch_rpy += 3
        Roll_Pitch_Yaw(Roll_rpy,Pitch_rpy,Yaw_rpy)
        time.sleep(0.1)
    while(pressed_key == '4'):
        Pitch_rpy -= 3
        Roll_Pitch_Yaw(Roll_rpy,Pitch_rpy,Yaw_rpy)
        time.sleep(0.1)
    while(pressed_key == '5'):
        Yaw_rpy += 3
        Roll_Pitch_Yaw(Roll_rpy,Pitch_rpy,Yaw_rpy)
        time.sleep(0.1)
    while(pressed_key == '6'):
        Yaw_rpy -= 3
        Roll_Pitch_Yaw(Roll_rpy,Pitch_rpy,Yaw_rpy)
        time.sleep(0.1)
    while(pressed_key == 'c'):
        pressed_key  = None
        # break
    while(pressed_key == 'p'):
        print("Stoping")
        time.sleep(0.35)
        all_To_initial(0.01)

count = 0
def on_press(key):
    global pressed_key, count
    #     pressed_key = str(key)
    pressed_key = key
    print(key)
    while(pressed_key == 'w'):
        if(count == 0):
            print("--------------------")
            print(f'Robot is moving: Forward')
            all_Move(25, 0, 0, 0.01)
            startTrot(60,0,0.01)
            count = 1
        elif(count == 1):
            Trot_gait(60, 0, 0.01)

    while(pressed_key == 's'):
        if(count == 0):
            print("--------------------")
            print(f'Robot is moving: Backward')
            all_Move(-20, 0, 0, 0.01)
            startTrot(-60,0,0.01)
            count = 1
        elif(count == 1):
            Trot_gait(-60, 0, 0.01)
    
    while(pressed_key == 'a'):
        if(count == 0):
            print("--------------------")
            print(f'Robot is moving: Left')
            all_Move(10, -20, 0, 0.01)
            startTrot(0, -40, 0.02)
            count = 1
        elif(count == 1):
            Trot_gait(0, -40, 0.01)

    while(pressed_key == 'd'):
        if(count == 0):
            print("--------------------")
            print(f'Robot is moving: Right')
            all_Move(10, 20, 0, 0.01)
            startTrot(0, 40, 0.02)
            count = 1
        elif(count == 1):
            Trot_gait(0, 40, 0.01) 

    while(pressed_key == 'p'):
        print("Stoping")
        time.sleep(0.35)
        all_To_initial(0.01)
        
    # while (pressed_key == 'c'):
    #     break

def on_press_2(key):
    global pressed_key
    pressed_key = key
    while(pressed_key == '9'):
        Turn(0)
    while(pressed_key == '0'):
        Turn(1)
    while(pressed_key == 'p'):
        print("Stoping")
        time.sleep(0.35)
        all_To_initial(0.01)

# def Walk_gait(SL):
#     A_x = -25;  A_z = -10
#     all_Move(SL,-30, 0, 0.01)
    
#     RR = posRR
#     for t in np.arange(0,1.005,0.25):
#         alpha= np.pi*(t)#(0 -> pi)
#         x = (RR[0]+(SL/2))/2 + ((RR[0]-(SL/2))/2)*np.cos(alpha)  #(1 -> 0 -> -1)
#         y = RR[1]
#         z = RH+SH*np.sin(alpha)     #(0 -> 1 -> 0)
#         RIGHT_Inverse_Kinematics(Rear,x,y,z)
#         time.sleep(0.025)
#     FR = posFR
#     for t in np.arange(0,1.005,0.25):
#         alpha= np.pi*(t)#(0 -> pi)
#         x = (FR[0]+(SL/2))/2 + ((FR[0]-(SL/2))/2)*np.cos(alpha)  #(1 -> 0 -> -1)
#         y = FR[1]
#         z = RH+SH*np.sin(alpha)     #(0 -> 1 -> 0)
#         RIGHT_Inverse_Kinematics(Front,x,y,z)
#         time.sleep(0.025)
        
#     all_Move(SL, 60, 0, 0.01)
        
#     RL = posRL
#     for t in np.arange(0,1.005,0.25):
#         alpha= np.pi*(t)#(0 -> pi)
#         x = (RL[0]+(SL/2))/2 + ((RL[0]-(SL/2))/2)*np.cos(alpha)  #(1 -> 0 -> -1)
#         y = RL[1]
#         z = RH+SH*np.sin(alpha)     #(0 -> 1 -> 0)
#         LEFT_Inverse_Kinematics(Rear,x,y,z)
#         time.sleep(0.025)
#     FL = posFL
#     for t in np.arange(0,1.005,0.25):
#         alpha= np.pi*(t)#(0 -> pi)
#         x = (FL[0]+(SL/2))/2 + ((FL[0]-(SL/2))/2)*np.cos(alpha)  #(1 -> 0 -> -1)
#         y = FL[1]
#         z = RH+SH*np.sin(alpha)     #(0 -> 1 -> 0)
#         LEFT_Inverse_Kinematics(Rear,x,y,z)
#         time.sleep(0.025)
        
#     all_Move(0,-30, 0, 0.01)
    
def startTrot(SL_x, SL_y, Time_delay):
    if(SL_x > 0):   A_x = -25;  A_z = -10   # forward
    elif(SL_x < 0): A_x = 20;   A_z = -5  # backward
    else:           A_x = -7;   A_z = 0
    
    if(SL_y > 0):   A_y = 10    # right
    elif(SL_y < 0): A_y = -10   # left
    else:           A_y = 0
    
    for t in np.arange(0.125,0.505,0.125):
        x = -SL_x*t + A_x
        z = RH 
        y = l1 + SL_y*(t) + A_y
        LEFT_Inverse_Kinematics(Front,x,y,z)
        z += A_z   
        y = -l1 + SL_y*(t) + A_y
        RIGHT_Inverse_Kinematics(Rear,x,y,z)

        alpha= np.pi*(2*t)
        x = -(SL_x/4)*np.cos(alpha) + A_x + SL_x/4
        z = RH + SH/1.5*np.sin(alpha)
        y = -l1 + (SL_y/4)*np.cos(alpha) + A_y - SL_y/4
        RIGHT_Inverse_Kinematics(Front,x,y,z)
        z += A_z
        y = l1 + (SL_y/4)*np.cos(alpha) + A_y - SL_y/4
        LEFT_Inverse_Kinematics(Rear,x,y,z)
        time.sleep(Time_delay)
        
def endTrot(SL_x, SL_y, Time_delay):
    if(SL_x > 0):   A_x = -25;  A_z = -10   # forward
    elif(SL_x < 0): A_x = 20;   A_z = -5  # backward
    else:           A_x = -7;   A_z = 0
    
    if(SL_y > 0):   A_y = 10    # right
    elif(SL_y < 0): A_y = -10   # left
    else:           A_y = 0
    
    for t in np.arange(0.125,0.505,0.125):
        x = -SL_x*t + A_x + SL_x/2
        z = RH 
        y = -l1 + SL_y*(t) + A_y - SL_y/2
        RIGHT_Inverse_Kinematics(Front,x,y,z)
        z += A_z   
        y = l1 + SL_y*(t) + A_y - SL_y/2
        LEFT_Inverse_Kinematics(Rear,x,y,z)

        alpha= np.pi*(2*t)
        x = -(SL_x/4)*np.cos(alpha) + A_x - SL_x/4
        z = RH + SH/1.5*np.sin(alpha)
        y = l1 + (SL_y/4)*np.cos(alpha) + A_y + SL_y/4
        LEFT_Inverse_Kinematics(Front,x,y,z)
        z += A_z
        y = -l1 + (SL_y/4)*np.cos(alpha) + A_y + SL_y/4
        RIGHT_Inverse_Kinematics(Rear,x,y,z)
        time.sleep(Time_delay)   

def Trot_gait(SL_x, SL_y, Time_delay):
    start = timeit.default_timer()
    # adjust center of mass
    if(SL_x > 0):   A_x = -25;  A_z = -10   # forward
    elif(SL_x < 0): A_x = 20;   A_z = -5  # backward
    else:           A_x = -7;   A_z = 0
    
    if(SL_y > 0):   A_y = 10    # right
    elif(SL_y < 0): A_y = -10   # left
    else:           A_y = 0

    for t in np.arange(0,1.005,0.125):
        x = -SL_x*t+(SL_x/2) + A_x
        y = -l1 + SL_y*(t)-(SL_y/2) + A_y
        z = RH 
        RIGHT_Inverse_Kinematics(Front,x,y,z)
        z += A_z
        y = l1 + SL_y*(t)-(SL_y/2) + A_y
        LEFT_Inverse_Kinematics(Rear,x,y,z)

        alpha= np.pi*(1-t)
        x = (SL_x/2)*np.cos(alpha) + A_x
        y = l1 - (SL_y/2)*np.cos(alpha) + A_y
        z = RH+SH*np.sin(alpha)
        LEFT_Inverse_Kinematics(Front,x,y,z)
        z += A_z
        y = -l1 - (SL_y/2)*np.cos(alpha) + A_y
        RIGHT_Inverse_Kinematics(Rear,x,y,z)
        time.sleep(Time_delay)
   
    for t in np.arange(0,1.005,0.125):
        alpha= np.pi*(1-t)
        x = (SL_x/2)*np.cos(alpha) + A_x
        y = -l1 - (SL_y/2)*np.cos(alpha) + A_y
        z = RH+SH*np.sin(alpha) 
        RIGHT_Inverse_Kinematics(Front,x,y,z)
        z += A_z
        y = l1 - (SL_y/2)*np.cos(alpha) + A_y
        LEFT_Inverse_Kinematics(Rear,x,y,z)
        
        x = -SL_x*(t)+(SL_x/2) + A_x
        y = l1 + SL_y*(t)-(SL_y/2) + A_y
        z = RH
        LEFT_Inverse_Kinematics(Front,x,y,z)
        z += A_z
        y = -l1 + SL_y*(t)-(SL_y/2) + A_y
        RIGHT_Inverse_Kinematics(Rear,x,y,z)
        time.sleep(Time_delay)
    stop = timeit.default_timer()
    print(stop - start, " (seconds)")
        
def Trot_C(SL_xleft, SL_xright, Time_delay):
    pass

def Turn(veer):
    for t in np.arange(0,1.005,0.25):
        if(veer == 0): 
            t =  1-t # dao chieu
        alpha= np.pi*(t)#(0 -> pi)
        x = (0+17.8)/2 + ((0-17.8)/2)*np.cos(alpha)  #(1 -> 0 -> -1)
        y = (l1+8.6)/2 + ((l1-8.6)/2)*np.cos(alpha) #(1 -> 0 -> -1)
        z = RH+SH*np.sin(alpha)     #(0 -> 1 -> 0)
        # print(x,y,z)
        LEFT_Inverse_Kinematics(Front,x,y,z)
        RIGHT_Inverse_Kinematics(Rear,-x,-y,z)
        x = -27*(1-t)
        y = -78.3 + (-l1+78.3)*t
        RIGHT_Inverse_Kinematics(Front, x, y, RH)
        LEFT_Inverse_Kinematics(Rear, -x, -y, RH)
        time.sleep(0.025)

    for t in np.arange(0,1.005,0.25):
        if(veer == 0): 
            t =  1-t # dao chieu
        alpha= np.pi*(t)#(0 -> pi)
        x = (0-27)/2 + (0-(-27)/2)*np.cos(alpha)  #(1 -> 0 -> -1)
        y = (-l1+(-78.3))/2 + ((-l1-(-78.3))/2)*np.cos(alpha) #(1 -> 0 -> -1)
        z = RH+SH*np.sin(alpha)     #(0 -> 1 -> 0)
        RIGHT_Inverse_Kinematics(Front,x,y,z)
        LEFT_Inverse_Kinematics(Rear,-x,-y,z)
        x = -17.8*(1-t)
        y = -8.5 + (-l1+8.5)*t
        RIGHT_Inverse_Kinematics(Rear, x, y, RH)
        LEFT_Inverse_Kinematics(Front, -x, -y, RH)
        time.sleep(0.025)
    

### Main code ###
# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

# Move servo on channel O between extremes.
# Calib()
initial_position()
listen_keyboard(on_press=on_press, delay_second_char=0.05)
# getRollPitch_MPU()

## Trot forward
# all_Move(25, 0, 0, 0.01)
# startTrot(60,0,0.01)

### Trot backward
# all_Move(-20, 0, 0, 0.01)
# startTrot(-60,0,0.01)

### Trot right
# all_Move(10, 20, 0, 0.01)
# startTrot(0, 40, 0.02)

### Trot left
# all_Move(10, -20, 0, 0.01)
# startTrot(0, -40, 0.02)

## 5 step forward
# all_Move(25, 0, 0, 0.01)
# startTrot(60,0,0.01)
# for _ in range(5):
#     Trot_gait(60, 0, 0.01)   
# endTrot(60, 0, 0.01) 
# all_Move(-25, 0, 0, 0.01)

# time.sleep(1)

## 5 step backward
# all_Move(-25, 0, 0, 0.01)
# startTrot(-60,0,0.01)
# for _ in range(5):
#     Trot_gait(-60, 0, 0.01)   
# endTrot(-60, 0, 0.01) 
# all_Move(25, 0, 0, 0.01)

# while True:
    # pass
    # roll = float(input('Nhap goc: '))
    # pitch = float(input('Nhap goc: '))
    # Roll_Pitch_Yaw(roll, pitch, 0)
    # Turn()
    
    # startTrot(0,-40,0.01)
    # time.sleep(1)
    # endTrot(0,-40,0.01)    
    # time.sleep(1)

    ### Dung ngoi (theo initial())
    # all_Move(0,0,-60, 0.025)
    # all_Move(0,0,60, 0.025)
    
    ### chom toi, lui (theo initial())
    # all_Move(30,0,0, 0.025)
    # all_Move(-30,0,0, 0.025)
    # all_Move(-30,0,0, 0.025)
    # all_Move(30,0,0, 0.025)
    
    ### chom trai, phai (theo initial())
    # all_Move(0,30,0, 0.025)
    # all_Move(0,-30,0, 0.025)
    # all_Move(0,-30,0, 0.025)
    # all_Move(0,30,0, 0.025)
    
    ### test all_To_initial()
    # print("------")
    # all_Move(-40,20,20, 0.01)
    # print("------")
    # time.sleep(1)
    # all_Move(0,-50,-30, 0.01)
    # # print(posFL)
    # # print(posFR)
    # # print(posRL)
    # # print(posRR)
    # print("------")
    # time.sleep(1)
    # all_To_initial(0.01)
    # time.sleep(1)
    
    ### forward
    # Trot_gait(60, 0, 0.01)  
      
    ### bachward
    # Trot_gait(-60, 0, 0.01)   
    
    ### move right
    # Trot_gait(0, 40, 0.01) 
    
    ### move left
    # Trot_gait(0, -40, 0.01)    
    
    ### multi direction
    # Trot_gait(40, -40, 0.01)  
    # Trot_gait(-40, 40, 0.01)

    # Trot_C(-60,-60,0.01)
    # exit


    

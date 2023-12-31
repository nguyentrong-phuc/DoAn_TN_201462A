
from __future__ import division
import time
import math as m
import numpy as np
import array as arr 
# Import the PCA9685 module.

import Adafruit_PCA9685

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 100  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

l1=46.4 #mm
l2=100 #mm
l3=100 #mm


#Define side
Front = 0
Rear = 1

#Define leg adress
front_left = 0
front_right = 3
rear_left = 6
rear_right = 9

#Define mode
sit = 10
stand = 11
move_forward = 12
move_right = 13
move_left = 14 # repairing...
move_around_CCW = 15 #nguoc chieu KDH
move_around_CW = 16 #cung chieu KDH
roll = 17
pitch = 18
yaw = 19

# Offset matrix
FL = arr.array('i', [95, 60, 0]) 
FR = arr.array('i', [90, 85, 165]) 
RL = arr.array('i', [95, 60, 10]) 
RR = arr.array('i', [90, 85, 165]) 

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


def initial_position():
    setLegAngles(front_left,0,45,90)
    setLegAngles(front_right,0,-45,-90)
    setLegAngles(rear_left,0,45,90)
    setLegAngles(rear_right,0,-45,-90)
    time.sleep(1.5)
    #exit()


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
    if( k2< l1**2 or k1> (l1**2 + (l2 + l3)**2)):
        print("The position :(",x,y,z,") is Out of workspace")
        exit()


def LEFT_Inverse_Kinematics(leg,x,y,z):
    #check work space
    Check_work_space(x,y,z)
    # Theta1
    alpha = m.acos(y/np.sqrt(y**2 + z**2))
    t1 = -m.acos(l1/np.sqrt(z**2 + y**2)) - alpha
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
    # Kiem tra dk theta3
    if (t3 < 0):
        print("Error Theta3")
        exit()
    # Theta2
    s2 = (A * (l2 + l3 * c3) + B * l3 * s3)  
    c2 = (B * (l2 + l3 * c3) - A * l3 * s3)  
    t2 = m.atan2(s2, c2)

    # Convert Rad -> Deg
    t1 = t1*180/np.pi
    t2 = t2*180/np.pi
    t3 = t3*180/np.pi
    
    if(leg == Rear):
        setLegAngles(rear_left,t1,t2,t3)
    elif(leg == Front):
        setLegAngles(front_left,t1,t2,t3)
        

def RIGHT_Inverse_Kinematics(leg,x,y,z):
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
    # Kiem tra dk theta3
    if (t3 > 0):
        print("Error Theta3")
        exit()
    # Theta2
    s2 = (A * (l2 + l3 * c3) + B * l3 * s3)  
    c2 = (B * (l2 + l3 * c3) - A * l3 * s3)  
    t2 = -m.atan2(s2, c2)
    
    # Convert Rad -> Deg
    t1 = t1*180/np.pi
    t2 = t2*180/np.pi
    t3 = t3*180/np.pi
    
    if(leg == Rear):
        setLegAngles(rear_right,t1,t2,t3)
    elif(leg == Front):
        setLegAngles(front_right,t1,t2,t3)
    

# Mode of pose
def Pose(mode):
    if(mode == move_right):
        for t in np.arange(0,2.1,0.1):
            if(t<=1):
                x = 0
                y = l1 +30*(t)-15
                z = -120
                LEFT_Inverse_Kinematics(Front,x,y,z)

                x = 0
                y = -l1+30*(t)-15 
                z = -120
                RIGHT_Inverse_Kinematics(Rear,x,y,z)

                alpha= np.pi*(t)
                x = 0
                y = -l1 + 15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                RIGHT_Inverse_Kinematics(Front,x,y,z)

                alpha= np.pi*(t)
                x = 0
                y = l1+15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                LEFT_Inverse_Kinematics(Rear,x,y,z)
                time.sleep(0.01)

            else:
                x = 0
                y = l1 +30*(t-1)-15
                z = -120
                LEFT_Inverse_Kinematics(Rear,x,y,z)

                x = 0
                y = -l1+30*(t-1)-15 
                z = -120
                RIGHT_Inverse_Kinematics(Front,x,y,z)

                alpha= np.pi*(2-t)
                x = 0
                y = -l1 -15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                RIGHT_Inverse_Kinematics(Rear,x,y,z)

                alpha= np.pi*(2-t)
                x = 0
                y = l1-15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                LEFT_Inverse_Kinematics(Front,x,y,z)

                time.sleep(0.01)
    
    elif(mode == move_left):
        for t in np.arange(0,2.1,0.1):
            if(t<=1):
                x = 0
                y = l1 -30*(t)+15
                z = -120
                LEFT_Inverse_Kinematics(front_left,x,y,z)
        
                x = 0
                y = -l1-30*(t)+15 
                z = -120
                RIGHT_Inverse_Kinematics(rear_right,x,y,z)

                alpha= np.pi*(t)
                x = 0
                y = -l1 - 15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                RIGHT_Inverse_Kinematics(front_right,x,y,z)
            
                alpha= np.pi*(t)
                x = 0
                y = l1-15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                LEFT_Inverse_Kinematics(rear_left,x,y,z)
                
            else:
                x = 0
                y = l1 -30*(t-1)+15
                z = -120
                LEFT_Inverse_Kinematics(rear_left,x,y,z)
        
                x = 0
                y = -l1-30*(t-1)+15 
                z = -120
                RIGHT_Inverse_Kinematics(front_right,x,y,z)

                alpha= np.pi*(2-t)
                x = 0
                y = -l1 + 15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                RIGHT_Inverse_Kinematics(rear_right,x,y,z)
            
                alpha= np.pi*(2-t)
                x = 0
                y = l1+15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                LEFT_Inverse_Kinematics(front_left,x,y,z)
                
    elif(mode == move_around_CW):
        for t in np.arange(0,2.1,0.1):
            if(t<=1):
                x = 0
                y = l1 +30*(t)-15
                z = -120
                LEFT_Inverse_Kinematics(Front,x,y,z)

                x = 0
                y = -l1-30*(t)+15 
                z = -120
                RIGHT_Inverse_Kinematics(Rear,x,y,z)

                alpha= np.pi*(t)
                x = 0
                y = -l1 + 15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                RIGHT_Inverse_Kinematics(Front,x,y,z)
            
                alpha= np.pi*(t)
                x = 0
                y = l1-15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                LEFT_Inverse_Kinematics(Rear,x,y,z)
                time.sleep(0.01)

            else:
                x = 0
                y = l1 -30*(t-1)+15
                z = -120
                LEFT_Inverse_Kinematics(Rear,x,y,z)
        
                x = 0
                y = -l1+30*(t-1)-15 
                z = -120
                RIGHT_Inverse_Kinematics(Front,x,y,z)

                alpha= np.pi*(2-t)
                x = 0
                y = -l1 +15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                RIGHT_Inverse_Kinematics(Rear,x,y,z)

                alpha= np.pi*(2-t)
                x = 0
                y = l1-15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                LEFT_Inverse_Kinematics(Front,x,y,z)

                time.sleep(0.01)
                
    elif(mode == move_around_CCW):
        for t in np.arange(0,2.1,0.1):
            if(t<=1):
                x = 0
                y = l1 +30*(t)-15
                z = -120
                LEFT_Inverse_Kinematics(Rear,x,y,z)

                x = 0
                y = -l1-30*(t)+15 
                z = -120
                RIGHT_Inverse_Kinematics(Front,x,y,z)

                alpha= np.pi*(t)
                x = 0
                y = -l1 + 15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                RIGHT_Inverse_Kinematics(Rear,x,y,z)

                alpha= np.pi*(t)
                x = 0
                y = l1-15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                LEFT_Inverse_Kinematics(Front,x,y,z)
                
                time.sleep(0.01)

            else:
                x = 0
                y = l1 -30*(t-1)+15
                z = -120
                LEFT_Inverse_Kinematics(Front,x,y,z)
        
                x = 0
                y = -l1+30*(t-1)-15 
                z = -120
                RIGHT_Inverse_Kinematics(Rear,x,y,z)

                alpha= np.pi*(2-t)
                x = 0
                y = -l1 + 15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                RIGHT_Inverse_Kinematics(Front,x,y,z)

                alpha= np.pi*(2-t)
                x = 0
                y = l1-15*np.cos(alpha)
                z = -120+15*np.sin(alpha)
                LEFT_Inverse_Kinematics(Rear,x,y,z)

                time.sleep(0.01)
                
    elif(mode == roll):
        input=int(); # angle
        for:
            move;
            
            
    elif(mode == pitch):
        input=int(); # angle
        for:
            move;
            
    elif(mode == yaw):
        input=int(); # angle
        for:
            move;
        
        
    elif(mode == move_forward):
        for t in np.arange(0,2.1,0.1):
            if(t<=1):
                x = -40*t+20
                y = l1 
                z = -120
                LEFT_Inverse_Kinematics(Rear,x,y,z)

                x = -40*t+20
                y = -l1 
                z = -120
                RIGHT_Inverse_Kinematics(Front,x,y,z)

                alpha= np.pi*(1-t)
                x = 20*np.cos(alpha)
                y = -l1 
                z = -120+20*np.sin(alpha)
                RIGHT_Inverse_Kinematics(Rear,x,y,z)

                alpha= np.pi*(1-t)
                x = 20*np.cos(alpha)
                y = l1 
                z = -120+20*np.sin(alpha)
                LEFT_Inverse_Kinematics(Front,x,y,z)
                time.sleep(0.01)

            else:
                x = -40*(t-1)+20
                y = l1 
                z = -120
                LEFT_Inverse_Kinematics(Front,x,y,z)

                x = -40*(t-1)+20
                y = -l1 
                z = -120
                RIGHT_Inverse_Kinematics(Rear,x,y,z)

                alpha= np.pi*(2-t)
                x = 20*np.cos(alpha)
                y = -l1 
                z = -120+20*np.sin(alpha)
                RIGHT_Inverse_Kinematics(Front,x,y,z)

                alpha= np.pi*(2-t)
                x = 20*np.cos(alpha)
                y = l1 
                z = -120+20*np.sin(alpha)
                LEFT_Inverse_Kinematics(Rear,x,y,z)
                time.sleep(0.01)
                
            
        
    elif(mode == sit): ## sit
        t1=0;t2=0;t3=0
        for t in np.arange(0,3.01,0.05):
            if(t<=1):
                t2=(45+25*t)
                t3=(90+50*t)
                setLegAngles(rear_left,t1,t2,t3)
                setLegAngles(front_left,t1,t2,t3)
                t2=-t2
                t3=-t3
                setLegAngles(front_right,t1,t2,t3)
                setLegAngles(rear_right,t1,t2,t3)
                
            elif(t <= 2):
                t2=(70-20*(t-1))
                setLegAngles(rear_left,t1,t2,t3)
                setLegAngles(front_left,t1,t2,t3)
                t2=-t2
                setLegAngles(front_right,t1,t2,t3)
                setLegAngles(rear_right,t1,t2,t3)
                
            elif(t <= 3):
                t2=(50-50*(t-2))
                t3=(140-50*(t-2))
                setLegAngles(rear_left,t1,t2,t3)
                setLegAngles(front_left,t1,t2,t3)
                t2=-t2
                t3=-t3
                setLegAngles(front_right,t1,t2,t3)
                setLegAngles(rear_right,t1,t2,t3)
                
            time.sleep(0.05)
                
    elif(mode == stand): ## stand 
        t1=0;t2=0;t3=0
        for t in np.arange(3,6.01,0.05):  
            if(t<=4):
                t2 = (50 * (t-3));
                t3 = (90 + 50 * (t-3));
                setLegAngles(rear_left,t1,t2,t3)
                setLegAngles(front_left,t1,t2,t3)
                t2=-t2
                t3=-t3
                setLegAngles(front_right,t1,t2,t3)
                setLegAngles(rear_right,t1,t2,t3)
                
            elif(t <= 5):
                t2 = (50 + 20 * (t - 4))
                setLegAngles(rear_left,t1,t2,t3)
                setLegAngles(front_left,t1,t2,t3)
                t2=-t2
                setLegAngles(front_right,t1,t2,t3)
                setLegAngles(rear_right,t1,t2,t3)
                
            else:
                t2 = (70 - 25 * (t - 5));
                t3 = (140 - 50 * (t - 5));
                setLegAngles(rear_left,t1,t2,t3)
                setLegAngles(front_left,t1,t2,t3)
                t2=-t2
                t3=-t3
                setLegAngles(front_right,t1,t2,t3)
                setLegAngles(rear_right,t1,t2,t3)
                
            time.sleep(0.05)

### Main code ###
# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
# Move servo on channel O between extremes.
initial_position()

while True:
    ### sit <-> stand ###
    # Pose(sit)
    # time.sleep(2)
    # Pose(stand)
    # time.sleep(2)
    
    ### move forward ###
    Pose(move_forward)
    
    


    

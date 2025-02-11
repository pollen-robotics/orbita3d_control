import csv
import numpy as np

fields = ['timestamp','torque_on','target_roll','target_pitch','target_yaw','velocity_limit_top','velocity_limit_middle','velocity_limit_bottom','torque_limit_top','torque_limit_middle','torque_limit_bottom']

#let's generate a simple sinus on yaw for 5s at 1kHz the very inneficient way
def generate_rpy(timestamp, roll, pitch, yaw, torque=True,vel_limit_top=1.0, vel_limit_middle=1.0, vel_limit_bottom=1.0, torque_limit_top=1.0,torque_limit_middle=1.0, torque_limit_bottom=1.0):
        #the velocity and torque limits are in 100% of the maximum allowed
        is_on= "false"
        if torque:
            is_on = "true"
        return [timestamp, is_on, roll, pitch, yaw, vel_limit_top, vel_limit_middle, vel_limit_bottom, torque_limit_top, torque_limit_middle, torque_limit_bottom]

AMP=np.radians(20.0) #amplitude of 20°
FREQ=1.0 #frequency of 1Hz

#the created file will be located where you launched jupyter
with open('test_sinus_input.csv', 'w') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(fields)
    
    for it in range(5000): #so it is in ms
        writer.writerow(generate_rpy(timestamp=it/1000.0,roll=0.0,pitch=0.0,yaw=np.sin(2.0*np.pi*FREQ*it/1000.0)))
        
        #let's generate a step, 0° until t=STEP_TIME then yaw=AMP
AMP=np.radians(20.0) #amplitude of 20°
STEP_TIME=1.0 #step will be on from 1s

#the created file will be located where you launched jupyter
with open('test_step_input.csv', 'w') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(fields)
    
    for it in range(5000): #so it is in ms
        writer.writerow(generate_rpy(timestamp=it/1000.0,roll=0.0,pitch=0.0,yaw=(AMP if it/1000.0>=STEP_TIME else 0.0)))
        
        
target_yaw = 0.0
with open('test_input.csv', 'w') as csvfile:   
    writer = csv.writer(csvfile)
    writer.writerow(fields)
    
    # a bit of time to stabilize 
    writer.writerow(generate_rpy(timestamp=0,roll=0.0,pitch=0.0,yaw=0))
    writer.writerow(generate_rpy(timestamp=0.1,roll=0.0,pitch=0.0,yaw=0))
    
    for it in range(500): #so it is in ms
        target_yaw = target_yaw + 2*np.pi/500.0
        writer.writerow(generate_rpy(timestamp=it/100.0+0.1,roll=0.0,pitch=0.0,yaw=target_yaw))
    
    for it in range(500): #so it is in ms
        target_yaw = target_yaw - 2*np.pi/500.0
        writer.writerow(generate_rpy(timestamp=(it+500)/100.0+0.1,roll=0.0,pitch=0.0,yaw=target_yaw))
        
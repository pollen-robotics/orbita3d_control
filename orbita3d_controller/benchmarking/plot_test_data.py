import time
import numpy as np
import sys
import pandas as pd

import pickle

# get the file name from the arguments
if len(sys.argv) != 2:
    print("Error: Invalid number of arguments")
    print("Usage: python plot_test_data.py <file_name>")
    sys.exit()

# Getting back the objects:
filename = sys.argv[1]
df = pd.read_csv(filename)
t = np.array(df['timestamp'])
pos = np.array([df['present_roll'],df['present_pitch'],df['present_yaw']]).T
pos_mot = np.array([df['present_pos_top'],df['present_pos_mid'],df['present_pos_bot']]).T
tar = np.array([df['target_roll'],df['target_pitch'],df['target_yaw']]).T
vel = np.array([df['present_velocity_top'],df['present_velocity_mid'],df['present_velocity_bot']]).T
torque = np.array([df['present_torque_top'],df['present_torque_mid'],df['present_torque_bot']]).T
axis_sensors = np.array([df['axis_sensor_top'],df['axis_sensor_mid'],df['axis_sensor_bot']]).T
n_axis = 3


print("Plotting")
import matplotlib.pyplot as plt

fig, axs = plt.subplots(3,n_axis, figsize=(10,10), sharex=True)

for i, a in enumerate(axs.T):
    a[0].step(t,pos_mot[:,i], label = "measured")
    a[1].step(t,vel[:,i], label = "measured")
    a[2].step(t, torque[:,i], label = "measured")
    a[2].step(t, np.ones_like(t)*100, 'k--', linewidth=2, label = "max allowed torque")
    a[2].step(t, np.ones_like(t)*-100, 'k--', linewidth=2)

axs[0,0].set_title("Top")
axs[0,1].set_title("Mid")
axs[0,2].set_title("Bot")


# set title of the figure overall
fig.suptitle("Motor variables during the test")

for i, a in enumerate(axs[:].T):
    if i == 0:
        a[0].set_ylabel("position [rad]")
        a[1].set_ylabel("velocity [rad/s]")
        a[2].set_ylabel("current [mA]")
    a[2].set_ylim([-200,200])
    a[0].grid()
    a[1].grid()
    a[2].grid()
a[0].legend()
a[1].legend()
a[2].legend()
    
plt.show()

def wrap(angle):
    return (angle + 2 * np.pi) % (2 * np.pi)

axis_sensors = axis_sensors
axis_readings_initial = np.array(axis_sensors[0,:]).reshape(-1,1)

#pos = pos/28.1715976331*28.0
axis_calc = pos_mot.T % (2*np.pi)
axis_calc = (wrap(axis_calc ))

axis_error = axis_calc -axis_sensors.T
for i, ax_e in enumerate(axis_error):
    for j,a in enumerate(ax_e):
        if np.abs(a) > np.pi:
            axis_error[i,j] = a - (np.sign(a))*2*np.pi


fig, axs = plt.subplots(3,n_axis, figsize=(10,10), sharex=True)

# add the title
fig.suptitle("Absolute position of the orbita and backlash axis")

for i, a in enumerate(axs.T):
    a[0].step(t, np.rad2deg(pos[:,i]), label="measured")
    a[0].step(t, np.rad2deg(tar[:,i]), label="target")    
    a[1].step(t, np.rad2deg(axis_sensors[:,i]), label = "actual [deg]")
    a[1].step(t, np.rad2deg(axis_calc[i,:]), label = "estimated [deg]")
    a[2].step(t, np.rad2deg(axis_error[i,:]), label = "backlash [deg]")

for i, a in enumerate(axs[:].T):
    if i == 0:
        a[0].set_ylabel("orbita position [deg]")
        a[2].set_ylabel("backlash axis position [deg]")
        a[1].set_ylabel("motor position [deg]")
    a[0].grid()
    a[1].grid()
    a[2].grid()
a[0].legend()
a[1].legend()
a[2].legend()



axs[0,0].set_title("Roll")
axs[0,1].set_title("Pitch")
axs[0,2].set_title("Yaw")

axs[1,0].set_title("Top")
axs[1,1].set_title("Mid")
axs[1,2].set_title("Bot")
    
plt.legend()

plt.legend()
plt.show()


# SCORBOT ER-VII Simulator (kinematics only)

from inspect import classify_class_attrs
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import math
import keyboard

# Constants related to the manipulator [mm]
L1 = 358.5  # base to shoulder
L2 = 300    # shoulder to elbow
L3 = 350    # elbow to pitch axis
L4 = 251    # pitch axis to gripper

# Initial angle values
# alpha, a, d, theta
angles = [0,np.pi/4,np.pi/2,-np.pi/6,0]

# angle increment for interaction
angle_inc = np.pi/180*10

# Initialize figure and plot
fig = plt.figure()
ax = plt.axes(projection='3d')
# remove some shortcuts
plt.rcParams['keymap.save'].remove('s')
plt.rcParams['keymap.fullscreen'].remove('f')



# Configure figure and plot
def init_plot():
    ax.set_xlim3d([-500, 500])
    ax.set_ylim3d([-500, 500])
    ax.set_zlim3d([0, 1000])


# Key press events
def key_pressed(event):
    print('key pressed: ', event.key)

    if event.key == 'a':
        angles[0] += angle_inc
    elif event.key == 'z':
        angles[0] -= angle_inc
    elif event.key == 's':
        angles[1] += angle_inc
    elif event.key == 'x':
        angles[1] -= angle_inc
    elif event.key == 'd':
        angles[2] += angle_inc
    elif event.key == 'c':
        angles[2] -= angle_inc
    elif event.key == 'f':
        angles[3] += angle_inc
    elif event.key == 'v':
        angles[3] -= angle_inc
    elif event.key == 'g':
        angles[4] += angle_inc
    elif event.key == 'b':
        angles[4] -= angle_inc
    else:
        return
    
    ax.clear()   

    # Initialize plot
    init_plot()

    # Update the arm
    p, T = update_arm()

    # Plot all joints
    plot_arm(p)


# set D-H parameters
def set_dh_params(angles):
    dh_params = np.array([[0, 0, L1, angles[0]],
                          [angles[1], 0, L2, 0],
                          [angles[2],0,L3,0],
                          [angles[3],0,L4,0],
                          [0,0,0,angles[4]]])
    return dh_params


# To calculate the D-H matrix
def dh_matrix(alpha, a, d, theta):
    T = np.array([[math.cos(theta),                   -math.sin(theta),                  0,                  a],
         [math.sin(theta)*math.cos(alpha), math.cos(theta)*math.cos(alpha),   -math.sin(alpha), -math.sin(alpha)*d],
         [math.sin(theta)*math.sin(alpha), math.cos(theta)*math.sin(alpha),    math.cos(alpha),  math.cos(alpha)*d],
         [                              0,                               0,                  0,                  1]])
    return T
   

# Denavit-Hartenberg (D-H) convention 
def get_transf(T, p, dh_params):
    print("D-H params:\n",dh_params)
    for f in range(0,len(dh_params[:])-1):
        print("----------------")
        print("Transform from ",f, " to ",f+1)
        T = np.matmul(T,dh_matrix(dh_params[f,0], dh_params[f,1], dh_params[f,2], dh_params[f,3]))
        print(T)
        p[:,f+1] = np.matmul(T,np.array([0,0,0,1]))

    return p
        

# Plot manipulator
def plot_arm(p):
    ax.plot3D(p[0,:], p[1,:], p[2,:], 'gray')
    ax.scatter(p[0,:], p[1,:], p[2,:], s=20)
    plt.show()

def update_arm():
    # Initialize transformation
    T = np.identity(4)

    # Tnitialize joints coordinates
    p = np.vstack((np.zeros((3,5),dtype=np.float32),np.ones((1,5),dtype=np.float32)))   
    
    print("values of p",p)
    
    # set D-H parameters
    dh_params = set_dh_params(angles)

    # Get all points
    p = get_transf(T, p, dh_params)

    # Show points
    print("----------------")
    print("Points")
    print(p)

    return p, T


def main():
    # define key press event
    fig.canvas.mpl_connect('key_press_event', key_pressed)

    # Initialize plot
    init_plot()

    # Update the arm
    p, T = update_arm()

    # Plot all joints
    plot_arm(p)


if __name__ == "__main__":
    main()
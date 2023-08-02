import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from mpl_toolkits.mplot3d import Axes3D
from utl.coordinate_transforms import *

def plot_states(*args):
    '''
    This function will plot the states that are supplied. 

    Each state should have a value that is a numpy array 

    Each dictionary provided needs to have:
        - key = states , value = dictionary
        - key = time,    value = array
        - key = name,    value  = string
    '''

    # Creating Plot for all Translational States
    fig, axs = plt.subplots(2,3)
    fig.suptitle('Plot of Translational States')
    for arg in args:
        x = arg['time']

        
        axs[0, 0].plot(x, arg['states']['x'][1:], label=arg['name'])
        axs[0, 0].set_title('North Position')
        axs[0, 0].legend()
        axs[0, 1].plot(x, arg['states']['y'][1:], label=arg['name'])
        axs[0, 1].set_title('East Position')
        axs[0, 1].legend()
        axs[0, 2].plot(x, arg['states']['z'][1:], label=arg['name'])
        axs[0, 2].set_title('Down Position')
        axs[0, 2].legend()

        axs[1, 0].plot(x, arg['states']['u'][1:], label=arg['name'])
        axs[1, 0].set_title('Body Velocity u')
        axs[1, 0].legend()
        axs[1, 1].plot(x, arg['states']['v'][1:], label=arg['name'])
        axs[1, 1].set_title('Body Velocity v')
        axs[1, 1].legend()
        axs[1, 2].plot(x, arg['states']['w'][1:], label=arg['name'])
        axs[1, 2].set_title('Body Velocity w')
        axs[1, 2].legend()

        for ax in axs.flat:
            ax.set(xlabel='Time', ylabel='Meters')

        for ax in axs.flat:
            ax.label_outer()

    fig, axs = plt.subplots(2,3)
    fig.suptitle('Plot of Rotational States')
    for arg in args:
        x = arg['time']

        axs[0, 0].plot(x, arg['states']['phi'][1:], label=arg['name'])
        axs[0, 0].set_title('Phi')
        axs[0, 0].legend()
        axs[0, 1].plot(x, arg['states']['theta'][1:], label=arg['name'])
        axs[0, 1].set_title('Theta')
        axs[0, 1].legend()
        axs[0, 2].plot(x, arg['states']['psi'][1:], label=arg['name'])
        axs[0, 2].set_title('Psi')
        axs[0, 2].legend()
        axs[1, 0].plot(x, arg['states']['p'][1:], label=arg['name'])
        axs[1, 0].set_title('p')
        axs[1, 0].legend()
        axs[1, 1].plot(x, arg['states']['q'][1:], label=arg['name'])
        axs[1, 1].set_title('q')
        axs[1, 1].legend()
        axs[1, 2].plot(x, arg['states']['r'][1:], label=arg['name'])
        axs[1, 2].set_title('r')
        axs[1, 2].legend()

        for ax in axs.flat:
            ax.set(xlabel='Time', ylabel='Meters')

        for ax in axs.flat:
            ax.label_outer()

    plt.show()


def plot3d(origin : np.ndarray, coord_frames : list, simDict : dict):
    '''
    This function will plot the different coordinate axes of the vehicle

    No translational motion will be displayed!! All origins are at 0.0, 0.0, 0.0

    The inertial frame of the vehicle is always displayed

    The first keyword argument is a numpy array specifying the origin of the coordinate frames
    X - origin[0], Y - origin[1], Z - origin[2]

    The second keyword argument is a LIST of STRINGS that specifies which axes the user wants displayed.
    The following are the options: 
        1. Vehicle  (NED)
        2. Body (post roll)
        3. Vehicle-1 (post yaw)
        4. Vehicle-2 (post pitch)

    Each simDict dictionary provided needs to have:
        - key = states , value = dictionary
        - key = time,    value = array
        - key = name,    value  = string

    ALL ARGUMENTS MUST BE FOR THE SAME LENGTH OF TIME!!!!
    '''
    
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    # Make the inertial NED coordinate: X = 1.0, Y = 1.0, Z = -1.0; Origin = 0.0
    X = origin[0]; Y = origin[1]; Z = origin[2]
    U = 1.0; V = 1.0; W = -1.0

    ax.set_xlim3d(X-2.0, X+2.0)
    ax.set_ylim3d(Y-2.0, Y+2.0)
    ax.set_zlim3d(Z-2.0, Z+2.0)


    ax.quiver(X, Y, Z, U, 0.0, 0.0, color='g', label='Inertial X')
    ax.quiver(X, Y, Z, 0.0, V, 0.0, color='b', label='Inertial Y')
    ax.quiver(X, Y, Z, 0.0, 0.0, W, color='r', label='Inertial Z')
    
    
    xVec_v = np.array([1.0, 0.0, 0.0])
    yVec_v = np.array([0.0, 1.0, 0.0])
    zVec_v = np.array([0.0, 0.0, -1.0])

    for coord_frame in coord_frames:

        if coord_frame == 'Vehicle-1':
            yaw = simDict['states']['psi'][10]    # psi == yaw
            dcm = dcm_vehicle_vehicle1(yaw)

            xVec_v1 = dcm @ xVec_v
            yVec_v1 = dcm @ yVec_v
            zVec_v1 = dcm @ zVec_v

            ax.quiver(X, Y, Z, xVec_v1[0], xVec_v1[1], xVec_v1[2], color='g', linestyle='dashed', label='Vehicle-1 X')
            ax.quiver(X, Y, Z, yVec_v1[0], yVec_v1[1], yVec_v1[2], color='b', linestyle='dashed', label='Vehicle-1 Y')
            ax.quiver(X, Y, Z, zVec_v1[0], zVec_v1[1], zVec_v1[2], color='r', linestyle='dashed', label='Vehicle-1 Z')


        elif coord_frame == 'Vehicle-2':
            yaw = simDict['states']['yaw']
            pitch = simDict['states']['pitch']
            v_to_v1  = dcm_vehicle_vehicle1(yaw)
            v1_to_v2 = dcm_vehicle1_vehicle2(pitch)

            
        elif coord_frame == 'Body':
            yaw   = simDict['states']['yaw']
            pitch = simDict['states']['pitch']
            roll  = simDict['states']['roll']
            v_to_v1  = dcm_vehicle_vehicle1(yaw)
            v1_to_v2 = dcm_vehicle1_vehicle2(pitch)
            v2_to_b  = dcm_vehicle2_body(roll)
            
        else:
            raise Exception('Please specify one of the following coordinate frames: Vehicle-1, Vehicle-2, or Body')



    


    ax.legend()

    plt.show()

# def update_quiver(num, Q, X, Y):
    
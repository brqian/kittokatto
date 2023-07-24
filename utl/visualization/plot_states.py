import numpy as np
import seaborn as sns
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from utl.coordinate_transforms import *

def plot_states(states : dict, time : np.ndarray):
    '''
    This function will plot the states that are supplied. 

    Each state should have a value that is a numpy array 
    '''
    sns.set()
    x = time
    pltCounter = 1
    for key, value in states.items():
        y = value
        
        plt.figure(pltCounter)
        plt.xlabel("Time [s]")
        plt.ylabel(key)
        plt.plot(x, y)

    plt.show()



def plot3d(states: dict):
    
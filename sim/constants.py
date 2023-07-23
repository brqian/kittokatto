import numpy as np


'''
In this file we will describe the physical parameters of the quadcopter drone. 

Assuming that the rotors are massless

Assuming that the motors are equal mass and are modeled as point masses

Assuming that the center of gravity is located perfectly in the middle of the quadcopter

Use: https://arxiv.org/pdf/2202.07021.pdf as a reference for what the physical parameters of the quadcopter should be. 
'''


motor_distance_from_center = 0.243 # [meters] This is assuming that the motor is 5 inches away from the center of mass

copter_mass = 1.587 # [kg] Total mass 

rotor_blade_radius = 0.02 # [meters] Assuming that the rotor blade radius is 2 cm 

moment_of_inertia_matrix = np.array([[0.0213, 0, 0][0, 0.02217, 0][0, 0, 0.0282]])  # [kg*m^2]


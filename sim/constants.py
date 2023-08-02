import numpy as np


'''
In this file we will describe the physical parameters of the quadcopter drone. 

Assuming that the rotors are massless

Assuming that the motors are equal mass and are modeled as point masses

Assuming that the center of gravity is located perfectly in the middle of the quadcopter

Use: https://arxiv.org/pdf/2202.07021.pdf as a reference for what the physical parameters of the quadcopter should be. 
'''


def get_motor_distance_from_center():
    motor_distance_from_center = 0.243 # [meters] This is assuming that the motor is 5 inches away from the center of mass
    return motor_distance_from_center

def get_rotor_blade_radius():
    rotor_blade_radius = 0.02 # [meters] Assuming that the rotor blade radius is 2 cm 
    return rotor_blade_radius

def get_moment_of_inertia():
    moment_of_inertia_matrix = np.array([[0.0213, 0, 0],[0, 0.02217, 0],[0, 0, 0.0282]])  # [kg*m^2]
    return moment_of_inertia_matrix

def get_quadcopter_mass():
    copter_mass = 1.587 # [kg] Total mass 
    return copter_mass

def get_thrust_coefficient():
    thrust_coefficient = 3.7102e-5 # [Ns^2]
    return thrust_coefficient

def get_drag_coefficient():
    drag_coefficient = 7.6933e-7 # [Nms^2]
    return drag_coefficient

def get_moment_arm():
    moment_arm = 0.243 # [m]
    return moment_arm

def get_max_propeller_speed():
    prop_max_speed = 494.27 # [rad/s]
    return prop_max_speed

def get_gravity():
    gravity = 9.81 # [m/s2]
    return gravity
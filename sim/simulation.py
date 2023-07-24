import numpy as np

from simpleDynamics import simpleDynamics 
from constants import *


totalMass          = get_quadcopter_mass()
momentOfInertiaMat = get_moment_of_inertia()

massProperties = dict()
massProperties['mass'] = totalMass
massProperties['MomentsOfInertia'] = momentOfInertiaMat

simpleDyn = simpleDynamics(massProperties)

# Initial Conditions
state = dict()
state['x'] = [0.0]
state['y'] = [0.0]
state['z'] = [0.0]
state['u'] = [0.0]
state['v'] = [0.0]
state['w'] = [0.0]
state['phi'] = [0.0]
state['theta'] = [0.0]
state['psi'] = [0.0]
state['p'] = [0.0]
state['q'] = [0.0]
state['r'] = [0.0]

# Forces
forces = np.array([50, 50, 50])

# Moments
moments = np.array([0, 0, 0])

time_start = 0.0
time_end   = 10.0
dt = 0.1

cnt = 0
for i in np.arange(time_start, time_end, dt):
    

    # Would call some analysis to determine thrust 

    # Would call some analysis to sum up forces and moments 

    # updating the state
    state = simpleDyn.update_states(state, forces, moments, dt, cnt)

    cnt = cnt + 1











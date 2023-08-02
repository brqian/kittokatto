import numpy as np

from utl.visualization.plot_states import plot_states, plot3d
from simpleDynamics import simpleDynamics 
from constants import *


totalMass          = get_quadcopter_mass()
momentOfInertiaMat = get_moment_of_inertia()

massProperties = dict()
massProperties['mass'] = totalMass
massProperties['MomentsOfInertia'] = momentOfInertiaMat

simpleDyn = simpleDynamics(massProperties)

# Initial Conditions for forward euler
FE_state = dict()
FE_state['x'] = [0.0]
FE_state['y'] = [0.0]
FE_state['z'] = [0.0]
FE_state['u'] = [0.0]
FE_state['v'] = [1.0]
FE_state['w'] = [0.0]
FE_state['phi'] = [0.0]
FE_state['theta'] = [0.0]
FE_state['psi'] = [0.0]
FE_state['p'] = [0.0]
FE_state['q'] = [0.0]
FE_state['r'] = [2.0]

# Initial Conditions for rk4
rk4_state = dict()
rk4_state['x'] = [0.0]
rk4_state['y'] = [0.0]
rk4_state['z'] = [0.0]
rk4_state['u'] = [0.0]
rk4_state['v'] = [1.0]
rk4_state['w'] = [0.0]
rk4_state['phi'] = [0.0]
rk4_state['theta'] = [0.0]
rk4_state['psi'] = [0.0]
rk4_state['p'] = [0.0]
rk4_state['q'] = [0.0]
rk4_state['r'] = [2.0]

time_start = 0.0
time_end   = 10.0
dt = 0.1
tVec = np.arange(time_start, time_end, dt)

# Forces
forces = np.zeros(tVec.shape + (3,))
forces[0:100, 0] = 100*np.sin(10*tVec[0:100])
# forces[0:5,:] = np.array([np.sin(0:5), 100, 100,])

# Moments
moments = np.zeros(tVec.shape + (3,))


cnt = 0
for i in tVec:
    

    # Would call some analysis to determine thrust 

    # Would call some analysis to sum up forces and moments 

    # updating the state using forward euler
    FE_state = simpleDyn.forward_euler(FE_state, forces, moments, dt, cnt)

    # updating the state using rk4
    rk4_state = simpleDyn.rk4(rk4_state, forces, moments, dt, cnt)

    cnt = cnt + 1 


rk4plot = dict()
rk4plot['states'] = rk4_state
rk4plot['time']   = tVec
rk4plot['name']   = 'RK4' 

feplot = dict()
feplot['states'] = FE_state
feplot['time']   = tVec
feplot['name']   = 'fe' 
# plot_states(feplot, rk4plot)
# plot_states(FE_state, tVec)

origin = np.array([0.0, 0.0, 0.0])

plot3d(origin, ['Vehicle-1'], rk4plot)

print('hi')








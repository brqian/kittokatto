import numpy as np
from simpleDynamics import simpleDynamics

def runge_kutta_4(simpleDyn, state, forces, moments, dt, cnt):

    state = simpleDyn.update_states(state, forces, moments, dt, cnt)




def k1fun(tn, yn):
    pass

def k2fun(tn_h_2, yn_k1h_2):
    pass

def k3fun(tn_h_2, yn_k2h_2):
    pass

def k4fun(tn_h, yn_k3h):
    pass
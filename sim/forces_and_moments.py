import numpy as np
from sim.constants import *


class forcesAndMoments():
    '''
    All forces and moments in this model will return a 3x1 array. 

    Forces  = [Fx, Fy, Fz]
    Moments = [Mx, My, Mz]
    
    '''

    def __init__(self):
        pass

    def gravity_model(self):
        '''
        Gravity Forces will be returned in the body frame
        '''
        


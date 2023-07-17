import numpy as np


def simpleDynamics(self, massProperties):
    '''
    Upon initialization of this class, provide a dictionary of the Mass Properties for the quadcopter.
    massProperties['mass'] = 100   # KG
    massProperties['MomentsOfInertia'] = (3x3) matrix providing Ixx, Iyy, Izz

    This is a class for a simplified dynamics model of a quadcopter drone.
    Input:
        Thrusts from the 4 motors 
        Torques from the 4 motors 
        Moments of Inertia of the quadcopter


    Output 12 states:
    x,y,z  in the inertial fixed frame 
    u,v,w  in the body frame
    psi (vehicle-1), theta (vehicle-2), phi(body)
    psi_dot(vehicle-1), theta_dot(vehicle-2), phi_body(body)
    p, q, r in the body frame
    '''

    def __init__(self, massProperties):

        # Unpacking the initial conditions provided and setting them as attributes of the simpleDynamics class
        for key in massProperties:
            setattr(self, key, massProperties[key])
        pass

        self.state_dict = dict()
     

    def update_states(self, state : dict, forces, moments, dt):
        '''
        Input:
        forces   (3x1) 
        moments  (3x1)
        state    (12x1)  At the current timestep

        Output:
        translational accelerations (body-frame) 
        angular accelerations       (body-frame)
        
        Reference Chapter 3 Small Unmanned Aircraft
        Equations 3.15 and 3.17
        '''

        mass = self.mass
        MoI  = self.MomentsOfInertia  # 3x3 matrix

        # Setting forces and moments variables
        fx = forces[0]
        fy = forces[1]
        fz = forces[2]
        l = moments[0]
        m = moments[1]
        n = moments[2]

        # Unpacking states dictionary
        x = state['x']
        y = state['y']
        z = state['z']
        u = state['u']
        v = state['v']
        w = state['w']
        phi   = state['phi']
        theta = state['theta']
        psi   = state['psi']
        p = state['p']
        q = state['q']
        r = state['r']

        Ixx = MoI[0,0]
        Iyy = MoI[1,1]
        Izz = MoI[2,2]
        Ixy = MoI[0,2]

        # Reference Equations 3.13 in Small Unmanned Aircraft
        gamma  = Ixx*Izz - Ixy**2
        gamma1 = -1*Ixy*(Ixx - Iyy + Izz)/gamma
        gamma2 = (Izz*(Izz - Iyy) + Ixy**2)/gamma
        gamma3 = Izz/gamma
        gamma4 = -1*Ixy/gamma
        gamma5 = (Izz - Ixx)/Iyy
        gamma6 = Ixy/Iyy
        gamma7 = ((Ixx - Iyy)*Ixx) + Ixy**2
        gamma8 = Ixx/gamma

        # Equation 3.15 in Small Unmanned Aircraft
        udot = r*v - q*w + (1/mass)*fx
        vdot = p*w - r*u + (1/mass)*fy
        wdot = q*u - p*v + (1/mass)*fz

        pdot = gamma1*p*q - gamma2*q*r + gamma3*l + gamma4*n
        qdot = gamma5*p*r - gamma6*(p**2 - r**2) + (1/Ixx)*mass
        rdot = gamma7*p*q - gamma1*q*r + gamma4*l + gamma8*n

        # Propagate forward the dynamics 
        u_new = u + udot*dt
        v_new = v + vdot*dt
        w_new = w + wdot*dt

        x_new = 

        







        
        


        

    




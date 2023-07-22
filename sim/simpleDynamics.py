import numpy as np
from utl.utilities import vehicle_body

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

        # Equation 3.15 in Small Unmanned Aircraft in Inertial Frame
        udot = r*v - q*w + (1/mass)*fx
        vdot = p*w - r*u + (1/mass)*fy
        wdot = q*u - p*v + (1/mass)*fz

        # Equation 3.17 in Small Unmanned Aircraft in Body Frame
        pdot = gamma1*p*q - gamma2*q*r + gamma3*l + gamma4*n
        qdot = gamma5*p*r - gamma6*(p**2 - r**2) + (1/Ixx)*m
        rdot = gamma7*p*q - gamma1*q*r + gamma4*l + gamma8*n

        # Equation 3.16 in Small Unmanned Aircraft producing Euler Angles from body frame rotation rates
        phi_dot   = p + np.sin(phi)*np.tan(theta)*q + np.cos(phi)*np.tan(theta)*r
        theta_dot = p*np.cos(phi) - r*np.sin(phi) 
        psi_dot = (np.sin(phi)/np.cos(theta))*q + (np.cos(phi)/np.cos(theta))*r 

        # Converting u,v,w body frame velocity to inertial frame
        R_vehicle2body = vehicle_body(psi, theta, phi)
        R_body2vehicle = np.transpose(R_vehicle2body)

        pvel = R_body2vehicle @ np.array([u, v, w]) 

        pveln = pvel[0]   # Velocity in the north direction 
        pvele = pvel[1]   # Velocity in the east direction
        pveld = pvel[2]   # Velocity in the down direction

        # Propagate forward the dynamics
        x_new = x + pveln*dt      # x expressed in inertial frame
        y_new = y + pvele*dt      # y expressed in inertial frame
        z_new = z + pveld*dt      # z expressed in inertial frame  

        u_new = u + udot*dt   # u expressed in body frame    
        v_new = v + vdot*dt   # v expressed in body frame
        w_new = w + wdot*dt   # w expressed in body frame

        phi_new = phi + phi_dot*dt        # phi expressed in the v2 frame
        theta_new = theta + theta_dot*dt  # theta expressed in the v1
        psi_new = psi + psi_dot*dt        # psi expressed in the v frame

        p_new = p + pdot*dt   # p expressed in body frame
        q_new = q + qdot*dt   # q expressed in body frame
        r_new = r + rdot*dt   # r expressed in body frame


        # Return state dictionary that contains updated states

        updated_states = dict()
        updated_states['x'] = x_new
        updated_states['y'] = y_new
        updated_states['z'] = z_new

        updated_states['u'] = u_new
        updated_states['v'] = v_new
        updated_states['w'] = w_new

        updated_states['phi'] = phi_new
        updated_states['theta'] = theta_new
        updated_states['psi'] = psi_new

        updated_states['p'] = p_new
        updated_states['q'] = q_new
        updated_states['r'] = r_new

        
        return updated_states






        
        


        

    




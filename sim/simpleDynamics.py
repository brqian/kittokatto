import numpy as np
from utl.coordinate_transforms import vehicle_body


class simpleDynamics():
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
     

    def update_states(self, state : dict, forces : np.ndarray, moments : np.ndarray, dt, cnt):
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
        x = state['x'][cnt]
        y = state['y'][cnt]
        z = state['z'][cnt]
        u = state['u'][cnt]
        v = state['v'][cnt]
        w = state['w'][cnt]
        phi   = state['phi'][cnt]
        theta = state['theta'][cnt]
        psi   = state['psi'][cnt]
        p = state['p'][cnt]
        q = state['q'][cnt]
        r = state['r'][cnt]

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
        state['x'].append(x_new)
        state['y'].append(y_new)
        state['z'].append(z_new)

        state['u'].append(u_new)
        state['v'].append(v_new)
        state['w'].append(w_new)

        state['phi'].append(phi_new)
        state['theta'].append(theta_new)
        state['psi'].append(psi_new)

        state['p'].append(p_new)
        state['q'].append(q_new)
        state['r'].append(r_new)

        
        return state








        
        


        

    




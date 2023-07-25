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

    def compute_rates(self, state_dict : dict):
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
        fx = self.forces[0]
        fy = self.forces[1]
        fz = self.forces[2]
        l = self.moments[0]
        m = self.moments[1]
        n = self.moments[2]

        # Unpacking states dictionary
        x = state_dict['x'][-1]
        y = state_dict['y'][-1]
        z = state_dict['z'][-1]
        u = state_dict['u'][-1]
        v = state_dict['v'][-1]
        w = state_dict['w'][-1]
        phi   = state_dict['phi'][-1]
        theta = state_dict['theta'][-1]
        psi   = state_dict['psi'][-1]
        p = state_dict['p'][-1]
        q = state_dict['q'][-1]
        r = state_dict['r'][-1]

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

        # Return a rate dictionary
        rate_dict = dict()
        rate_dict['x'] = pveln
        rate_dict['y'] = pvele
        rate_dict['z'] = pveld
        rate_dict['u'] = udot
        rate_dict['v'] = vdot
        rate_dict['w'] = wdot
        rate_dict['phi'] = phi_dot
        rate_dict['theta'] = theta_dot
        rate_dict['psi'] = psi_dot        
        rate_dict['p'] = pdot
        rate_dict['q'] = qdot
        rate_dict['r'] = rdot
        
        return rate_dict
        
    def propagate_dynamics(self, state_dict : dict, rate_dict : dict, dt):
        
        updated_states = dict()
        
        # Looping over state and rate dictionary and checking for matching keys
        for (stateKey, stateVal), (rateKey, rateVal) in zip(state_dict.items(), rate_dict.items()):
            if stateKey == rateKey:
                updated_states[stateKey] = [float(stateVal[-1] + rateVal*dt)]

        return updated_states


    def forward_euler(self, state : dict, forces : np.ndarray, moments : np.ndarray, dt):
        
        self.forces  = forces
        self.moments = moments 

        # Obtain the rates
        rates = self.compute_rates(state)

        # Propagate the dynamics
        updated_states = self.propagate_dynamics(state, rates, dt)

        # Looping over state and rate dictionary and checking for matching keys
        for (stateKey, stateVal), (updatedKey, updatedVal) in zip(state.items(), updated_states.items()):
            if stateKey == updatedKey:
                state[stateKey].append(updatedVal[0])
 
        return state

    def rk4(self, state : dict, forces : np.ndarray, moments : np.ndarray, dt):
        
        self.forces  = forces
        self.moments = moments 
        
        # Call to obtain rates 
        k1rates = self.compute_rates(state)

        k2states = self.propagate_dynamics(state, k1rates, dt/2)
        k2rates  = self.compute_rates(k2states)

        k3states = self.propagate_dynamics(state, k2rates, dt/2)
        k3rates  = self.compute_rates(k3states)

        k4rates  = self.propagate_dynamics(state, k3rates, dt)

        # Compute next time step
        for key in state.keys():
            newVal = float(state[key][-1] + (dt/6)*(k1rates[key] + 2*k2rates[key] + 2*k3rates[key] + k4rates[key]))
            state[key].append(newVal)

        return state







        
        


        

    




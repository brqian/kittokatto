import numpy as np


'''
This file contains the definition of various Direction Cosine Matrices

Function name should start with originating frame and end with desired frame.

Example: 

dcm_body_inertial - This is a DCM that transforms a 3x1 vector in the body frame to a 3x1 vector in the inertial frame

q_body2inertial - This is a quaternion that transforms a 3x1 vector in the body frame to a 3x1 vector in the inertial frame

'''

def dcm_vehicle_vehicle1(psi):
    '''
    This corresponds the yaw maneuver
    '''
    dcm = np.array([[np.cos(psi),  np.sin(psi), 0]
                    [-np.sin(psi), np.cos(psi), 0]
                    [0,            0,           1]])
    
    return dcm

def dcm_vehicle1_vehicle2(theta):
    '''
    This corresponds to the pitch maneuver
    '''
    dcm = np.array([[np.cos(theta),  0, -np.sin(theta)]
                    [0,              1,              0]
                    [np.sin(theta),  0,  np.cos(theta)]])
    
    return dcm    

def dcm_vehicle2_body(phi):
    '''
    This corresponds to the roll maneuver
    '''
    dcm = np.array([[1,              0,              0]
                    [0,    np.cos(phi),    np.sin(phi)]
                    [0,   -np.sin(phi),    np.cos(phi)]])
    
    return dcm        


def vehicle_body(psi, theta, phi):
    dcm_v2_body = dcm_vehicle2_body(phi)        # phi   - roll
    dcm_v1_v2   = dcm_vehicle1_vehicle2(theta)  # theta - pitch
    dcm_v_v1    = dcm_vehicle_vehicle1(psi)     # psi   - yaw

    # Based on the psi-theta-phi rotation sequence
    dcm = dcm_v2_body @ dcm_v1_v2 @ dcm_v_v1

def dcm_2_quaternion(dcm):
    '''
    This is assuming that the FIRST entry of the quaternion is scalar

    Using this website as a source: https://stevendumble.com/attitude-representations-understanding-direct-cosine-matrices-euler-angles-and-quaternions/
    '''

    r11 = dcm[0,0]
    r22 = dcm[1,1]
    r33 = dcm[2,2]

    r12 = dcm[0,1]
    r13 = dcm[0,2]

    r21 = dcm[1,0]
    r23 = dcm[1,2]

    r31 = dcm[2,0]
    r32 = dcm[2,1]

    q = np.zeros((4,))  # Assuming that the first entry is the scalar portion of the quaternion

    if (r22 > -1*r33) and (r11 > -1*r22) and (r11 > -1*r33):
        q[0] = np.sqrt( 1 + r11 + r22 + r33 )
        q[1] = (r23 - r32) / q[0]
        q[2] = (r31 - r13) / q[0]
        q[3] = (r12 - r21) / q[0]

    elif (r22 < -1*r33) and (r11 > r22) and (r11 > r33):
        q[1] = np.sqrt( 1 + r11 - r22 - r33 )
        q[0] = (r23 - r32) / q[1]
        q[2] = (r12 - r21) / q[1]
        q[3] = (r31 - r13) / q[1]

    elif (r22 > r33) and (r11 < r22) and (r11 < -1*r33):
        q[2] = np.sqrt( 1 - r11 + r22 - r33 )
        q[0] = (r31 - r13) / q[2]
        q[1] = (r12 - r21) / q[2]
        q[3] = (r23 - r32) / q[2]

    elif (r22 < r33) and (r11 < -1*r22) and (r11 < r33):
        q[3] = np.sqrt( 1 - r11 - r22 + r33 )
        q[0] = (r12 - r21) / q[3]
        q[1] = (r31 - r13) / q[3]
        q[2] = (r23 - r32) / q[3]
    else:
        raise ValueError('Something is not right with the DCM')
    
    q = (0.5)*q

    return q

def quaternion_2_dcm(q):
    '''
    Inputs:
        - q: must be a vector of shape (4,) where the first element is the scalar portion of the quaternion. Must also be a unit quaternion


    Output:
        - direction cosine matrix corresponding the to the quaternion input
    '''

    if np.linalg.norm(q) > 1.0:
        raise ValueError('Not a unit quaternion!')
    
    dcm = np.zeros((3,3))

    dcm[0,0] = q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2
    dcm[0,1] = 2*q[1]*q[2] + 2*q[0]*q[3]
    dcm[0,2] = 2*q[1]*q[3] - 2*q[0]*q[2]

    dcm[1,0] = 2*q[1]*q[2] - 2*q[0]*q[3]
    dcm[1,1] = q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2
    dcm[1,2] = 2*q[2]*q[3] + 2*q[0]*q[1]

    dcm[2,0] = 2*q[1]*q[3] + 2*q[0]*q[2]
    dcm[2,1] = 2*q[2]*q[3] - 2*q[0]*q[1]
    dcm[2,2] = q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2

    return dcm

def euler_2_quaternion(roll, pitch, yaw):
    '''
    Inputs:
        roll  (phi)   - euler angle corresponds to the amount of roll
        pitch (theta) - euler angle corresponds to the amount of pitch 
        yaw   (psi)   - euler angle corresponds to the amount of yaw

        Applied in the following manner: 
        R(roll) * R(pitch) * R(yaw)


    Outputs: 
        - a unit quaternion that chains the successive single axis rotations:
            q = q(roll) * q(pitch) * q(yaw)

        - q has shape of (4,)
    '''

    q = np.zeros((4,))

    q[0] = np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
    q[1] = np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
    q[2] = np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)
    q[3] = np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)

    return q

def quaternion_multiply(q1, q2):
    '''
    Inputs:
        - Both q1 and q2 are quaternions where the first compoenent is the scalar part, followed by they 3 imaginary axes. 
        - Shape of q1 and q2 are (4,)
    
    Outputs:
        - Outputs a (4,) quaternion where the first component is a scalar part followed by the 3 imaginary axes.

    Reference Wikipedia: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternions
    Section: Quaternions
    '''

    s = q1[0]    # scalar portion of q1
    v = q1[1:3]  # vector portion of q1

    t = q2[0]    # scalar portion of q2
    w = q2[1:3]  # vector portion of q2

    q = np.zeros((4,))

    q[0]   = s*t - np.dot(v,w) 
    q[1:3] = s*w + t*v + np.cross(v,w)

    return q



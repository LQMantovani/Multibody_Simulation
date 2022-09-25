import numpy as np
import math


def skew_s(a):
    #Returns the skew_symmetric matrix from a vector
    return np.array([[0, -a[2], a[1]],[a[2], 0, -a[0]],[-a[1], a[0], 0]])



def quat2rot(quat):
    #Function to generate a rotation matrix from a quaternion vector (last component is the real component)
    rq = quat[3]
    iq = np.array([[quat[0]], [quat[1]], [quat[2]]])
    Sq = skew_s(iq.flatten())
    return ((rq**2 - np.matmul(np.transpose(iq),iq))*np.identity(3) + 2*np.matmul(iq,np.transpose(iq)) - 2*rq*Sq)


def rot2quat(A):
    #Function that use an rotation matrix to output a quaternion (real component in the last one)
    #Based on: Tewari, A. Atmospheric and Space FLight Dynamic, 2007. ISBN: 978-0-8176-4437-6
    T = A.trace()
    qsq = np.array([[1+2*A[0,0]-T], [1+2*A[1,1]-T], [1+2*A[2,2]-T], [1+T]])/4
    x,i = np.max(qsq), np.argmax(qsq)
    q = np.array([0,0,0,0], dtype='float')
    q[i] = np.sqrt(x)
    if i == 3:
        q[0] = (A[1,2]-A[2,1])/(4*q[i])
        q[1] = (A[2,0]-A[0,2])/(4*q[i])
        q[2] = (A[0,1]-A[1,0])/(4*q[i])
        
    if i == 2:
        q[0] = (A[0,2]+A[2,0])/(4*q[i])
        q[1] = (A[2,1]+A[1,2])/(4*q[i])
        q[3] = (A[0,1]-A[1,0])/(4*q[i])
        
    if i == 1:
        q[0] = (A[0,1]+A[1,0])/(4*q[i])
        q[2] = (A[2,1]+A[1,2])/(4*q[i])
        q[3] = (A[2,0]-A[0,2])/(4*q[i])
        
    if i == 0:
        q[1] = (A[0,1]+A[1,0])/(4*q[i])
        q[2] = (A[0,2]+A[2,0])/(4*q[i])
        q[3] = (A[1,2]-A[2,1])/(4*q[i])
        
    return q


def ang2rot(phi, theta, psi):
    #Function that generates a rotation matrix from Euler Angles using the 3-2-1 rotation sequence
    #Angles should be in radians
    A_x = np.transpose(np.array([[1, 0, 0],[0, np.cos(phi), -np.sin(phi)],[0, np.sin(phi), np.cos(phi)]]))
    A_y = np.transpose(np.array([[np.cos(theta), 0, np.sin(theta)],[0, 1, 0],[-np.sin(theta), 0, np.cos(theta)]]))
    A_z = np.transpose(np.array([[np.cos(psi), -np.sin(psi), 0],[np.sin(psi), np.cos(psi), 0],[0, 0, 1]]))
    
    return np.matmul(A_z, np.matmul(A_y, A_x))
    
    
def dThetadt(quat, omega):
    #Function to obtain the quaternions time derivative
    p = omega[0]
    q = omega[1]
    r = omega[2]
    OMEGA = np.array([[0, r, -q, p],[-r, 0, p, q],[q, -p, 0, r],[-p, -q, -r, 0]])
    
    return 0.5*np.matmul(OMEGA,quat)
    
    
def quat2angle(quat):
    #Function to obtain Euler Angles in the 3-2-1 sequence from quaternions (last component is real)
    q0 = quat[3]
    q1 = quat[0]
    q2 = quat[1]
    q3 = quat[2]
    
    phi = np.arctan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1+q2*q2))
    theta = np.arcsin(2*(q0*q2-q3*q1))
    psi = np.arctan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
    
    return phi, theta, psi


def OrthogonalVector(a):
    #Function to obtain a vector perpendicular to vector a, assuming both vector have 3 components. Code adapted from: https://math.stackexchange.com/questions/137362/how-to-find-perpendicular-vector-to-another-vector
    x = a[0]
    y = a[1]
    z = a[2]
    
    s = np.linalg.norm(a)
    
    g = math.copysign(s, z)
    
    h = z + g
    
    vector = np.array([g*h - x*x, -x*y, -x*h])
    
    return vector/np.linalg.norm(vector)


def vec2rot(x, y):
    #Function to obtain a rotation matrix that goes from vector x to vector y
    
    return np.identity(3) + np.matmul(y, np.transpose(x)) - np.matmul(x, np.transpose(y)) + (1/(1 + np.dot(np.transpose(x), y)))*np.matmul((np.matmul(y, np.transpose(x)) - np.matmul(x, np.transpose(y))), (np.matmul(y, np.transpose(x)) - np.matmul(x, np.transpose(y))))


def vec2angle(a,b):
    #Returns a rotation matrix that goes from vector a to b based on vectors a and b
    M = np.array([[-a[2,0], a[1,0]],[a[1,0], a[2,0]]])
    return np.matmul(np.linalg.inv(M), np.array([[b[1,0]],[b[2,0]]]))


def angle_joints(i_a, i_b, i_c, j_a, AIi, AIj):
    #Function to obtain the angle between two body around a joint
    #Inputs:
    #a_i: joint direction (aligned with joint axis x) and written in the i body frame
    #b_i: joint vector perpendicular to a and wirtten in the i body frame
    #c_i: joint vector perpendicular to a and b wirtten in the i body frame
    #j_a: joint direction (aligned with joint axis x) and written in the j body frame
    #AIi: rotation matrix from the inertial system to body i
    #AIj: rotation matrix from the inertial system to body j
    #
    #Returns:
    #angle: angle between vectors (as seen from body i)
    
    #Body i vectors
    i_x = np.array([[1],[0],[0]])
    i_y = np.array([[0],[1],[0]])
    i_z = np.array([[0],[0],[1]])
    
    #Rotation matrix from the body frame to the joint frame
    i_Ra = vec2rot(i_x, i_a)
    #Rotation from the joint frame to the vector b
    i_Rb = vec2rot(i_a, i_b)
    #Rotation from the joint frame to the vector c
    i_Rc = vec2rot(i_a, i_c)
    
    #Body j vectors
    j_x = np.array([[1],[0],[0]])
    j_y = np.array([[0],[1],[0]])
    j_z = np.array([[0],[0],[1]])
    
    j_Ra = vec2rot(j_x, j_a)
    
    #b: joint vector perpendicular to a and wirtten in the j body frame
    j_b = np.matmul(i_Rb, j_a)
    #c: joint vector perpendicular to a and b wirtten in the j body frame
    j_c = np.matmul(i_Rc, j_a)
    
    j_ba = np.matmul(j_Ra, j_b)
    
    #i_b vector written in the reference frame of joint j
    i_bj = np.matmul(j_Ra, np.matmul(AIj, np.matmul(np.transpose(AIi), i_b)))
    
    #sine and cossine of the angle between vectors
    #invert the vectors to obtain the angle as seen by body j
    sin,cos = vec2angle(j_ba, i_bj)
    
    #angle between vectors (as seen from body i)
    angle = np.arctan2(sin,cos)*180/np.pi
    
    return angle



def SplitData(body, y):
    #Funcion to decompose the vector y in other vectors

    if body.name == "ECI":
        r = np.zeros(3)
        v = np.zeros(3)
        omega = np.zeros(3)
        A = np.identity(3)

    else:
        if body.N_state[0] > 0:
            r = y[body.pos_r0]
            v = y[body.pos_v0]
        else:
            r = np.zeros(3)
            v = r
            
        if body.N_state[1] > 0:
            omega = y[body.pos_omega0]
            A = np.transpose(quat2rot(y[body.pos_Theta0]))
        else:
            omega = np.zeros(3)
            A = np.identity(3)

    return r, v, omega, A
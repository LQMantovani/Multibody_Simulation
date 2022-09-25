import numpy as np 
from scipy.integrate import ode
from base_functions import *

#File that define classes for bodies and constraints used in simulation

class Body_rigid():
    #Class of rigid bodies
    def __init__(self, name, mass, inertia):
        #name: name of the body (string)
        #mass: mass of the body (int)
        self.name = name #Name of the body
        self.mass = mass #Mass of the body
        self.inertia = inertia #Inertia matrix of the body
        self.MASS = np.identity(3)*mass #Mass matrix of the body
        self.n_states = 6 #Number of states the body uses: obtained from d(X)/dt
        self.pos_r = None #Position of the position coordinates in the global states vector
        self.pos_v = None #Position of the velocity coordinates in the global states vector
        self.pos_omega = None #Position of the angular velocity coordinates in the global states vector
        self.pos_Theta = None #Position of the quaternions coordinates in the global states vector
        self.X_0 = np.zeros(13, dtype='float') #Initial conditions state vector (13 states for a rigid body)
        self.N_state = np.array([3, 3, 0]) #Number os position coordinates, number of angular velocity coordinates, number of flexible modes coordinates
        self.pos_v0 = np.array([0, 1, 2])
        self.pos_omega0 = np.array([3, 4, 5])
        self.pos_r0 = np.array([6, 7, 8])
        self.pos_Theta0 = np.array([9, 10, 11, 12])

        if self.name == "ECI":
            #Used as an inertial body in some constraints
            self.pos_r = []
            self.pos_v = []
            self.pos_omega = []
            self.pos_Theta = []
            self.X_0 = np.zeros(13, dtype='float')
            self.N_state = np.array([0, 0, 0])
            self.pos_v0 = []
            self.pos_omega0 = []
            self.pos_r0 = []
            self.pos_Theta0 = []
        
    def info(self):
        #Method to print body data
        print("Rigid body")
        print("Body name: " + self.name)
        print("Body mass: " + str(self.mass))
        print("Body inertia: " + str(self.inertia))
        
    def set_X_0(self, v_0, omega_0, r_0, Theta_0):
        #Method to update the body position
        if len(v_0) == 3:
            self.X_0[0:3] = v_0
        if len(omega_0) == 3:
            self.X_0[3:6] = omega_0
        if len(r_0) == 3:
            self.X_0[6:9] = r_0
        if len(Theta_0) == 4:
            self.X_0[9:13] = Theta_0
        
        
class Body_Mass():
    #Class of bodies that have mass but no inertia. Represents a point mass
    def __init__(self, name, mass):
        #name: name of the body
        #mass: mass of the body
        self.name = name #Name of the body
        self.mass = mass #Mass of the body
        self.MASS = np.identity(3)*mass #Mass matrix of the body
        self.n_states = 3 #Number of states the body uses: obtained from d(X)/dt
        self.pos_r = None #Position of the position coordinates in the global states vector
        self.pos_v = None #Position of the velocity coordinates in the global states vector
        self.pos_omega = [] #Position of the angular velocity coordinates in the global states vector
        self.pos_Theta = [] #Position of the quaternions coordinates in the global states vector
        self.X_0 = np.zeros(6) #Initial conditions state vector (6 states for a rigid body)
        self.N_state = np.array([3, 0, 0]) #Number os position coordinates, number of angular velocity coordinates, number of flexible modes coordinates
        self.pos_v0 = np.array([0, 1, 2])
        self.pos_omega0 = []
        self.pos_r0 = np.array([3, 4, 5])
        self.pos_Theta0 = []
        
    def info(self):
        #Method to print the body data
        print("Mass body")
        print("Body name: " + self.name)
        print("Body mass: " + self.mass)
        
    def set_X_0(self, v_0, r_0):
        #Method to update the body position
        if len(v_0) == 3:
            self.X_0[0:3] = v_0
        if len(r_0) == 3:
            self.X_0[3:6] = r_0
            
            
class Body_Inertia():
    #Class of bodies that have inertia but no mass
    def __init__(self, name, inertia):
        #name: name of the body
        #inertia: inertia of the body
        self.name = name #Name of the body
        self.inertia = inertia
        self.n_states = 3 #Number of states the body uses: obtained from d(X)/dt
        self.pos_r = [] #Position of the position coordinates in the global states vector
        self.pos_v = [] #Position of the velocity coordinates in the global states vector
        self.pos_omega = None #Position of the angular velocity coordinates in the global states vector
        self.pos_Theta = None #Position of the quaternions coordinates in the global states vector
        self.X_0 = np.zeros(7) #Initial conditions state vector (7 states for a rotating body)
        self.N_state = np.array([0, 3, 0]) #Number os position coordinates, number of angular velocity coordinates, number of flexible modes coordinates
        self.pos_v0 = []
        self.pos_omega0 = np.array([0, 1, 2])
        self.pos_r0 = []
        self.pos_Theta0 = np.array([3, 4, 5, 6])
        
    def info(self):
        #Method to print the body data
        print("Inertia body")
        print("Body name: " + self.name)
        print("Body inertia: " + self.mass)
        
    def set_X_0(self, omega_0, Theta_0):
        #Method to update the body position
        if len(omega_0) == 3:
            self.X_0[0:3] = omega_0
        if len(Theta_0) == 4:
            self.X_0[4:7] = Theta_0
        
        
class Constraint_r():
    #Constraints three transational degrees of freedom
    def __init__(self, i, j, p_i, p_j, k):
        #i: Body 1 object
        #j: Body j object
        #p_i: joint position in body i reference frame
        #p_j, joint position in body j reference frame
        #k: Gain value to be used in consatrint control
        self.body_i = i
        if j == "ECI" or j.name == "ECI":
            self.body_j = Body_rigid("ECI", 0, 0)
        else:
            self.body_j = j
        self.pos_i = p_i
        self.pos_j = p_j
        self.n_states = 3
        self.k = k
        
    #def C_matrices(self, r_i, omega_i, r_j, omega_j, A_i, A_j, v_i, v_j):
    def C_matrices(self, y_i, y_j):
        #y_i: state vector of body i
        #y_j: state vector of body j

        r_i, v_i, omega_i, A_i = SplitData(self.body_i, y_i)
        r_j, v_j, omega_j, A_j = SplitData(self.body_j, y_j)

        #r_i: inertial position vector of body i
        #omega_i: angular velocity of body i with respect to the inertial frame written in the body i reference frame
        #r_j: inertial position vector of body j
        #A_i: rotation matrix from body i reference frame to the inertial frame
        #omega_j: angular velocity of body j with respect to the inertial frame written in the body j reference frame
        #A_j: rotation matrix from body j reference frame to the inertial frame

        Cq_i = np.concatenate((np.identity(3), -np.matmul(A_i, skew_s(self.pos_i))), axis=1)
        Cq_j = np.concatenate((-np.identity(3), np.matmul(A_j, skew_s(self.pos_j))), axis=1)
        Cqf = -np.matmul(A_i, np.matmul(skew_s(omega_i), np.matmul(skew_s(self.pos_i), omega_i))) + np.matmul(A_j, np.matmul(skew_s(omega_j), np.matmul(skew_s(self.pos_j), omega_j)))
        
        Cq = np.concatenate((Cq_i, Cq_j), axis=1)
        qp = np.concatenate((v_i, omega_i, v_j, omega_j))
        
        Cqf_C = -2*self.k*np.matmul(Cq, qp) - (self.k**2)*self.C_vector(y_i, y_j)
        
        return Cq_i, Cq_j, -Cqf+Cqf_C
    
    def C_vector(self, y_i, y_j):
        #Method to calculate the constraint value:

        r_i, v_i, omega_i, A_i = SplitData(self.body_i, y_i)
        r_j, v_j, omega_j, A_j = SplitData(self.body_j, y_j)

        return (r_i + np.matmul(A_i, self.pos_i) - (r_j + np.matmul(A_j, self.pos_j)))
    
    def info(self):
        print("Translational constraint. Connects body i and j.")
        print("Body i: " + str(self.body_i.name))
        print("Body j: " + str(self.body_j.name))
        print("Joint position in body i frame: " + str(self.pos_i))
        print("Joint position in body j frame: " + str(self.pos_j))
        
        
        
class Constraint_q_fixed():
    #Constraints three rotational degrees of freedom
    def __init__(self, i, j, k, method):
        #i: Body 1 object
        #j: Body j object
        #k: Gain to perform constraint control
        #method: Method to calculate the constraint equation
        self.body_i = i
        if j == "ECI" or j.name == "ECI":
            self.body_j = Body_rigid("ECI", 0, 0)
        else:
            self.body_j = j
        self.method = method
        self.k = k
        self.stored = 0
        self.A_i = []
        self.A_j = []
        
        
    def info(self):
        print("Rotational constraint. Connects body i and j.")
        print("Body i: " + str(self.body_i.name))
        print("Body j: " + str(self.body_j.name))
        print("Joint position in body i frame: " + str(self.pos_i))
        print("Joint position in body j frame: " + str(self.pos_j))
        
    def C_matrices(self, y_i, y_j):

        r_i, v_i, omega_i, A_i = SplitData(self.body_i, y_i)
        r_j, v_j, omega_j, A_j = SplitData(self.body_j, y_j)

        #A_i: rotation matrix from body i reference frame to the inertial frame
        #A_j: rotation matrix from body j reference frame to the inertial frame
        #omega_j: angular velocity of body j with respect to the inertial frame written in the body j reference frame
        #omega_i: angular velocity of body i with respect to the inertial frame written in the body i reference frame
        if self.method  == 0:
            Cq_i = np.concatenate((np.zeros((3,3)), A_i), axis=1)
            Cq_j = np.concatenate((np.zeros((3,3)), -A_j), axis=1)
            #Cq_i = np.concatenate((np.zeros((3,3)), np.identity(3)), axis=1)
            #Cq_j = np.concatenate((np.zeros((3,3)), -np.identity(3)), axis=1)
            Cqf = np.zeros(3)
            Cqf_C = np.zeros(3)
            
        elif self.method == 1:
            
            if self.stored == 0:
                #runs just one time to allocate the vectors
                self.A_i = np.transpose(quat2rot(self.body_i.X_0[9:13]))
                self.A_j = np.transpose(quat2rot(self.body_j.X_0[9:13]))
                j_x = np.array([[1], [0], [0]])
                j_y = np.array([[0], [1], [0]])
                j_z = np.array([[0], [0], [1]])
                
                Aji = np.matmul(np.transpose(self.A_i), self.A_j)
                
                i_x = np.matmul(Aji, j_y)
                i_y = np.matmul(Aji, j_z)
                i_z = np.matmul(Aji, j_x)
                
                self.j_xs = skew_s(j_x.flatten())
                self.j_ys = skew_s(j_y.flatten())
                self.j_zs = skew_s(j_z.flatten())
                self.i_xs = skew_s(i_x.flatten())
                self.i_ys = skew_s(i_y.flatten())
                self.i_zs = skew_s(i_z.flatten())
                
                self.j_x = j_x
                self.j_y = j_y
                self.j_z = j_z
                self.i_x = i_x
                self.i_y = i_y
                self.i_z = i_z
                
                
                self.stored = 1
                
            
            Cq_i = np.concatenate((np.zeros((3,3)), np.concatenate((-np.matmul(np.transpose(self.j_x), np.matmul(np.transpose(A_j),np.matmul(A_i, self.i_xs))), -np.matmul(np.transpose(self.j_y), np.matmul(np.transpose(A_j),np.matmul(A_i, self.i_ys))), -np.matmul(np.transpose(self.j_z), np.matmul(np.transpose(A_j),np.matmul(A_i, self.i_zs)))), axis=0)), axis=1)
            
            Cq_j = np.concatenate((np.zeros((3,3)), np.concatenate((-np.matmul(np.transpose(self.i_x), np.matmul(np.transpose(A_i),np.matmul(A_j, self.j_xs))), -np.matmul(np.transpose(self.i_y), np.matmul(np.transpose(A_i),np.matmul(A_j, self.j_ys))), -np.matmul(np.transpose(self.i_z), np.matmul(np.transpose(A_i),np.matmul(A_j, self.j_zs)))), axis=0)), axis=1)
            
            Cqf_x = np.matmul(-np.matmul(np.transpose(self.j_x), np.matmul(np.transpose(A_j), np.matmul(A_i, skew_s(omega_i)))) + np.matmul(np.transpose(omega_j), np.matmul(np.transpose(self.j_xs), np.matmul(np.transpose(A_j), A_i))), np.matmul(self.i_xs, omega_i)) + np.matmul(-np.matmul(np.transpose(self.i_x), np.matmul(np.transpose(A_i), np.matmul(A_j, skew_s(omega_j)))) + np.matmul(np.transpose(omega_i), np.matmul(np.transpose(self.i_xs), np.matmul(np.transpose(A_i), A_j))), np.matmul(self.j_xs, omega_j))
            Cqf_y = np.matmul(-np.matmul(np.transpose(self.j_y), np.matmul(np.transpose(A_j), np.matmul(A_i, skew_s(omega_i)))) + np.matmul(np.transpose(omega_j), np.matmul(np.transpose(self.j_ys), np.matmul(np.transpose(A_j), A_i))), np.matmul(self.i_ys, omega_i)) + np.matmul(-np.matmul(np.transpose(self.i_y), np.matmul(np.transpose(A_i), np.matmul(A_j, skew_s(omega_j)))) + np.matmul(np.transpose(omega_i), np.matmul(np.transpose(self.i_ys), np.matmul(np.transpose(A_i), A_j))), np.matmul(self.j_ys, omega_j))
            Cqf_z = np.matmul(-np.matmul(np.transpose(self.j_z), np.matmul(np.transpose(A_j), np.matmul(A_i, skew_s(omega_i)))) + np.matmul(np.transpose(omega_j), np.matmul(np.transpose(self.j_zs), np.matmul(np.transpose(A_j), A_i))), np.matmul(self.i_zs, omega_i)) + np.matmul(-np.matmul(np.transpose(self.i_z), np.matmul(np.transpose(A_i), np.matmul(A_j, skew_s(omega_j)))) + np.matmul(np.transpose(omega_i), np.matmul(np.transpose(self.i_zs), np.matmul(np.transpose(A_i), A_j))), np.matmul(self.j_zs, omega_j))
            
            Cqf = -np.concatenate((Cqf_x, Cqf_y, Cqf_z))
            
            C = self.C_vector(y_i, y_j)
            Cq = np.concatenate((Cq_i, Cq_j), axis=1)
            
            #Method to obtain the terms associated with the constraint control
            Cqf_C = -(self.k**2)*C - 2*self.k*np.matmul(Cq, np.concatenate((np.zeros(3), omega_i, np.zeros(3), omega_j), axis=0))
            
        return Cq_i, Cq_j, Cqf+Cqf_C
    
    def C_vector(self, y_i, y_j):
        #Method to calculate the constraint value

        r_i, v_i, omega_i, A_i = SplitData(self.body_i, y_i)
        r_j, v_j, omega_j, A_j = SplitData(self.body_j, y_j)
        
        if self.method == 0:
            return np.matmul(A_i, omega_i) - np.matmul(A_j, omega_j)
        
        elif self.method == 1:
            return np.concatenate((np.matmul(np.transpose(np.matmul(A_i, self.i_x)), np.matmul(A_j, self.j_x)), np.matmul(np.transpose(np.matmul(A_i, self.i_y)), np.matmul(A_j, self.j_y)), np.matmul(np.transpose(np.matmul(A_i, self.i_z)), np.matmul(A_j, self.j_z))), axis=0).flatten()
            
            
class Constraint_2_theta():
    #Constraints two rotational degrees of freedom
    def __init__(self, i, j, a_i, a_j, k):
        #i: body i
        #J: body j
        #a_i: vector around the rotation can occur in body i reference frame
        #a_j: vector around the rotation can occur in body j reference frame
        #k: Constraint control gain
        self.body_i = i
        if j == "ECI" or j.name == "ECI":
            self.body_j = Body_rigid("ECI", 0, 0)
        else:
            self.body_j = j
        self.a_i = a_i/np.linalg.norm(a_i)
        self.a_j = a_j/np.linalg.norm(a_j) #Vector always parallel to a_i in the inertial frame
        self.b_j = OrthogonalVector(a_j) #Orthonormal vector to a_j
        self.c_j = np.cross(a_j, self.b_j) #Orthonormal vector to both a_j and b_j
        self.k = k
        self.n_states = 2
        self.a_is = skew_s(a_i)
        self.b_js = skew_s(self.b_j)
        self.c_js = skew_s(self.c_j)
        
        self.a_i = self.a_i.reshape((3,1))
        self.a_j = self.a_j.reshape((3,1))
        self.b_j = self.b_j.reshape((3,1))
        self.c_j = self.c_j.reshape((3,1))
        
        
    def info(self):
        print("Allows rotation around one axis. Connects body i and j.")
        print("Body i: " + str(self.body_i.name))
        print("Body j: " + str(self.body_j.name))
        print("Joint direction in body i frame: " + str(self.a_i))
        print("Joint direction in body j frame: " + str(self.a_j))
        
    def C_matrices(self, y_i, y_j):

        r_i, v_i, omega_i, A_i = SplitData(self.body_i, y_i)
        r_j, v_j, omega_j, A_j = SplitData(self.body_j, y_j)

        #A_i: rotation matrix from body i reference frame to the inertial frame
        #A_j: rotation matrix from body j reference frame to the inertial frame
        #omega_j: angular velocity of body j with respect to the inertial frame written in the body j reference frame
        #omega_i: angular velocity of body i with respect to the inertial frame written in the body i reference frame
        Cq_i = np.concatenate((np.zeros((2,3)), np.concatenate((-np.matmul(np.transpose(self.b_j), np.matmul(np.transpose(A_j),np.matmul(A_i, self.a_is))), -np.matmul(np.transpose(self.c_j), np.matmul(np.transpose(A_j),np.matmul(A_i, self.a_is)))), axis=0)), axis=1)
            
        Cq_j = np.concatenate((np.zeros((2,3)), np.concatenate((-np.matmul(np.transpose(self.a_i), np.matmul(np.transpose(A_i),np.matmul(A_j, self.b_js))), -np.matmul(np.transpose(self.a_i), np.matmul(np.transpose(A_i),np.matmul(A_j, self.c_js)))), axis=0)), axis=1)
        
        Cqf_b = np.matmul(-np.matmul(np.transpose(self.b_j), np.matmul(np.transpose(A_j), np.matmul(A_i, skew_s(omega_i)))) + np.matmul(np.transpose(omega_j), np.matmul(np.transpose(self.b_js), np.matmul(np.transpose(A_j), A_i))), np.matmul(self.a_is, omega_i)) + np.matmul(-np.matmul(np.transpose(self.a_i), np.matmul(np.transpose(A_i), np.matmul(A_j, skew_s(omega_j)))) + np.matmul(np.transpose(omega_i), np.matmul(np.transpose(self.a_is), np.matmul(np.transpose(A_i), A_j))), np.matmul(self.b_js, omega_j))
        
        Cqf_c = np.matmul(-np.matmul(np.transpose(self.c_j), np.matmul(np.transpose(A_j), np.matmul(A_i, skew_s(omega_i)))) + np.matmul(np.transpose(omega_j), np.matmul(np.transpose(self.c_js), np.matmul(np.transpose(A_j), A_i))), np.matmul(self.a_is, omega_i)) + np.matmul(-np.matmul(np.transpose(self.a_i), np.matmul(np.transpose(A_i), np.matmul(A_j, skew_s(omega_j)))) + np.matmul(np.transpose(omega_i), np.matmul(np.transpose(self.a_is), np.matmul(np.transpose(A_i), A_j))), np.matmul(self.c_js, omega_j))
        
        Cqf = -np.concatenate((Cqf_b, Cqf_c))
            
        C = self.C_vector(y_i, y_j)
        Cq = np.concatenate((Cq_i, Cq_j), axis=1)
           
        #Vector to obtain the terms associated with the constrait control
        Cqf_C = -(self.k**2)*C - 2*self.k*np.matmul(Cq, np.concatenate((np.zeros(3), omega_i, np.zeros(3), omega_j), axis=0))
            
        return Cq_i, Cq_j, Cqf+Cqf_C
        
        
    def C_vector(self, y_i, y_j):

        r_i, v_i, omega_i, A_i = SplitData(self.body_i, y_i)
        r_j, v_j, omega_j, A_j = SplitData(self.body_j, y_j)

        #Method to calculate the constraint vector
        return np.concatenate((np.matmul(np.transpose(self.a_i), np.matmul(np.transpose(A_i), np.matmul(A_j, self.b_j))), np.matmul(np.transpose(self.a_i), np.matmul(np.transpose(A_i), np.matmul(A_j, self.c_j)))), axis=0).flatten()
        
        
class Constraint_fixed():
    #Constraint that restrict three translational and three rotational degrees of freedom
    def __init__(self, i, j, p_i, p_j, method, k_r, k_q):
        #i: Body 1 object
        #j: Body j object
        #p_i: joint position in body i reference frame
        #p_j, joint position in body j reference frame
        #method: constraint equation to be used to constraint rotational degrees of freedom
        #k_r: constraint control value to be used in translational control
        #k_q: constraint contorl value to be used in rotatinal control
        self.body_i = i
        self.method = method
        if j == "ECI" or j.name == "ECI":
            self.body_j = Body_rigid("ECI", 0, 0)
            self.method = 0
        else:
            self.body_j = j
        self.pos_i = p_i
        self.pos_j = p_j
        self.k_r = k_r
        self.k_q = k_q
        self.constraint_r = Constraint_r(i, j, p_i, p_j, k_r)
        self.constraint_q_fixed = Constraint_q_fixed(i, j, k_q, method)
        self.n_states = 6
        
    def info(self):
        print("Translational and rotational constraint. Connects body i and j.")
        print("Body i: " + str(self.body_i.name))
        print("Body j: " + str(self.body_j.name))
        print("Joint position in body i frame: " + str(self.pos_i))
        print("Joint position in body j frame: " + str(self.pos_j))
        
    def C_matrices(self, y_i, y_j):
        Cq_i1, Cq_j1, Cqf1 = self.constraint_r.C_matrices(y_i, y_j)
        Cq_i2, Cq_j2, Cqf2 = self.constraint_q_fixed.C_matrices(y_i, y_j)   
        
        return np.concatenate((Cq_i1, Cq_i2), axis=0), np.concatenate((Cq_j1, Cq_j2), axis=0), np.concatenate((Cqf1, Cqf2), axis=0)
    
    def C_vector(self, y_i, y_j):
        #Method to calculate the constraint vector
        return np.concatenate((self.constraint_r.C_vector(y_i, y_j), self.constraint_q_fixed.C_vector(y_i, y_j)), axis=0)
        
        
class Constraint_Revolute_Joint():
    #Constraint that restricts three translational and two rotational degrees of freedom
    def __init__(self, i, j, p_i, p_j, a_i, a_j, k_r, k_q):
        #i: Body 1 object
        #j: Body j object
        #p_i: joint position in body i reference frame
        #p_j, joint position in body j reference frame
        #a_i: vector around the rotation can occur in body i reference frame
        #a_j: vector around the rotation can occur in body j reference frame
        #k_r: constraint control value to be used in translational control
        #k_q: constraint contorl value to be used in rotatinal control
        self.body_i = i
        if j == "ECI" or j.name == "ECI":
            self.body_j = Body_rigid("ECI", 0, 0)
            self.method = 0
        else:
            self.body_j = j
        self.pos_i = p_i
        self.pos_j = p_j
        self.a_i = a_i
        self.a_j = a_j
        self.k_r = k_r
        self.k_q = k_q
        self.constraint_r = Constraint_r(i, j, p_i, p_j, k_r)
        self.constraint_2_theta = Constraint_2_theta(i, j, a_i, a_j, k_q)
        self.n_states = 5
        
    def info(self):
        print("Revolute joint constraint. Connects body i and j.")
        print("Body i: " + str(self.body_i.name))
        print("Body j: " + str(self.body_j.name))
        print("Joint position in body i frame: " + str(self.pos_i))
        print("Joint position in body j frame: " + str(self.pos_j))
        print("Joint direction in body i frame: " + str(self.a_i))
        print("Joint direction in body j frame: " + str(self.a_j))
        
    def C_matrices(self, y_i, y_j):
        Cq_i1, Cq_j1, Cqf1 = self.constraint_r.C_matrices(y_i, y_j)
        Cq_i2, Cq_j2, Cqf2 = self.constraint_2_theta.C_matrices(y_i, y_j)   
        
        return np.concatenate((Cq_i1, Cq_i2), axis=0), np.concatenate((Cq_j1, Cq_j2), axis=0), np.concatenate((Cqf1, Cqf2), axis=0)
    
    def C_vector(self, y_i, y_j):
        #Method to calculate the constraint vector
        return np.concatenate((self.constraint_r.C_vector(y_i, y_j), self.constraint_2_theta.C_vector(y_i, y_j)), axis=0)
        
        
        
class System():
    #Class composed of bodies and constraints
    def __init__(self, name):
        #Name: name of the system (string)
        self.name = name
        self.bodies = []
        self.constraints = []
        self.n_bodies = 0
        self.n_constraints = 0
        self.n_states = 0
        self.n_constrained = 0
        self.X_0 = []
        self.DOF = 0
        self.set_control = False
        self.set_external = False
        
    def info(self):
        print("System's name: " + self.name)
        print("Number of bodies in the system: " + str(len(self.bodies)))
        print("Number of states in the system: " + str(self.n_states))
        print("Number of constraints in the system: " + str(self.n_constraints))
        print("Number of Degrees of Freedom in the system: " + str(self.DOF - self.n_constrained))
        print("Number of Degrees of Freedom constrained in the system: " + str(self.n_constrained))
        print("List of bodies in the system: ")
        j = 0
        for i in self.bodies:
            print("Bodies number " + str(j))
            i.info()
            j = j + 1
        j = 0
        for i in self.constraints:
            print("Constraint number" + str(j))
            i.info()
            j = j + 1
        
    def add_body(self, body):
        #Method to add a body to the system
        if body in self.bodies:
            print("Body already in system")
        else:
            self.bodies.append(body)
            print("Body added")
            self.n_bodies = self.n_bodies + 1
            self.state_vector()
        
    def del_body(self, body):
        #Method to remove body from the system
        if body in self.bodies:
            self.bodies.remove(body)
            print("Body " + str(body.name) + " removed")
            self.n_bodies = self.n_bodies - 1
            self.state_vector()
        else:
            print("Body is not in the system")
        
    def add_constraint(self, constraint):
        #Method to add a constraint to the system
        if constraint in self.constraints:
            print("Constraint already in the system")
        else:
            self.constraints.append(constraint)
            self.n_constraints = self.n_constraints + 1
            self.constrained_degrees()
        
    def del_constraint(self, constraint):
        #Method to remove constraint from the system
        if constraint in self.contraints:
            self.constraints.remove(constraint)
            print("Constraint removed")
            self.n_constraints = self.n_constraints - 1
            self.constrained_degrees()
        else:
            print("Contraint is not in the system")
        
    def state_vector(self):
        #Method to allocate the state vector
        self.n_states = 0
        self.DOF = 0
        for i in self.bodies:
            temp = self.n_states - 1
            if i.N_state[0] > 0:
                i.pos_v = np.array([self.n_states, self.n_states + 1, self.n_states + 2])
                temp = self.n_states + 2
            
            if i.N_state[1] > 0:
                i.pos_omega = np.array([temp + 1, temp + 2, temp + 3])
                
            self.n_states += i.n_states
            self.DOF += i.n_states
        for i in self.bodies:
            if i.N_state[0] > 0:
                i.pos_r = np.array([self.n_states, self.n_states + 1, self.n_states + 2])
                self.n_states += 3
        for i in self.bodies:
            if i.N_state[1] > 0:
                i.pos_Theta = np.array([self.n_states, self.n_states + 1, self.n_states + 2, self.n_states + 3])
                self.n_states += 4
        self.initialize_state_vector()
           
    def constrained_degrees(self):
        #Method to define the ammount of constrained degrees of freedom in the system
        self.n_constrained = 0
        for i in self.constraints:
            self.n_constrained = self.n_constrained + i.n_states
            
    def initialize_state_vector(self):
        #Method to allocate the variables in the state vector:
        self.X_0 = np.zeros(self.n_states)
        for body in self.bodies:
            if body.N_state[0] > 0:
                self.X_0[np.concatenate((body.pos_v,  body.pos_r))] = body.X_0[np.concatenate((body.pos_v0, body.pos_r0))]
            if body.N_state[1] > 0:
                self.X_0[np.concatenate((body.pos_omega, body.pos_Theta))] = body.X_0[np.concatenate((body.pos_omega0, body.pos_Theta0))]
            
    
    def Set_Control(self, function):
        #Method to assigned a personilized control function
        self.Control_Func = function
        self.set_control = True
        
    def Set_External(self, function):
        #Method to assigned a personilized external forces function
        self.External_Forces_Func = function
        self.set_external = True
        
    def Control(self, t, y):
        #Method to generate the control law
        if self.set_control:
            return self.Control_Func(self, t, y)
        else:
            return np.zeros(len(y))
        
    def External_Forces(self, t, y):
        #Method to generate the external forces in the system
        if self.set_external:
            return self.External_Forces_Func(self, t, y)
        else:
            return np.zeros(len(y))
        
        
        
class World():
    #World that encompass systems with bodies. It performs simulations
    def __init__(self, name):
        self.name = name
        self.systems = []
        
    def add_system(self, system):
        #Method to add a system to the world
        if system in self.systems:
            print("System already in World")
        else:
            self.systems.append(system)
            print("System added")
        
    def del_system(self, system):
        #Method to remove body from the system
        if system in self.systems:
            self.systems.remove(system)
            print("System " + str(system.name) + " removed")
        else:
            print("System is not in the world")
            
    def Simulate(self, t0, tf, dt):
        #Method to perform the simulation
        #to: initial time
        #tf: final time
        
        for system in self.systems:
            system.initialize_state_vector()
        
        solver = ode(self.diff_X)
        solver.set_integrator('vode', method='bdf', atol=1e-8, rtol=1e-8, max_step=1e-4)
        X0 = np.array([])
        for system in self.systems:
            #For now it only supports one system in the world
            X0 = np.concatenate((X0, system.X_0))
        solver.set_initial_value(X0, t0)
        solver.set_f_params(self)
        
        step = 1
        n_steps = np.int32(np.ceil((tf-t0)/dt))
        
        Y = np.zeros((n_steps + 1, len(X0)))
        T = np.zeros((n_steps + 1,1))
        Y[0,:] = X0
        T[0] = t0
        while solver.successful() and step < n_steps + 1:
            solver.integrate(solver.t + dt)
            T[step] = solver.t
            Y[step] = solver.y
            step += 1
            print("Simulation " + str(step/(n_steps+1)*100) + "% complete")
            
        return T, Y
            
            
    def diff_X(self, t, y, world):
        #Function to obtain the differential equations:
        dy = np.zeros(len(y))
        for system in world.systems:
            #System's generalized mass matrix with the constraint equations
            M = np.zeros((system.DOF + system.n_constrained, system.DOF + system.n_constrained))
            #System's generalized force matrix
            Q = np.zeros(system.DOF + system.n_constrained)
            Q_CONTROL = system.Control(t, y)
            Q_EXTERNAL = system.External_Forces(t, y)
            #States second and first time derivatives
            
            for body in system.bodies:
                
                
                #Generalized mass matriz and force vectors
                if body.N_state[0] > 0:
                    #For a body with translation coordinates
                    M[body.pos_v[0]:body.pos_v[-1]+1, body.pos_v[0]:body.pos_v[-1]+1] = body.mass*np.identity(3)
                    Q[body.pos_v[0]:body.pos_v[-1]+1] = Q_EXTERNAL[body.pos_v] + Q_CONTROL[body.pos_v]
                    dy[body.pos_r] = y[body.pos_v]
                    
                if body.N_state[1] > 0:
                    #For a body with rotation coordinates
                    M[body.pos_omega[0]:body.pos_omega[-1]+1, body.pos_omega[0]:body.pos_omega[-1]+1] = body.inertia
                    Q[body.pos_omega[0]:body.pos_omega[-1]+1] = -np.cross(y[body.pos_omega], np.matmul(body.inertia, y[body.pos_omega])) + Q_EXTERNAL[body.pos_omega] + Q_CONTROL[body.pos_omega]
                    
                    #Rotation matrix from the inertial frame to the body frame
                    #AI2B = quat2rot(quat)
                    
                    #Obtains the quaternions' time derivative
                    quat = y[body.pos_Theta]
                    quat = quat/np.linalg.norm(quat)
                    
                    dy[body.pos_Theta] = dThetadt(quat, y[body.pos_omega])
                
                
                
                
            
            pos_c = system.DOF
            for constraint in system.constraints:

                y_i = np.zeros(len(constraint.body_i.X_0))
                y_j = np.zeros(len(constraint.body_j.X_0))
                if constraint.body_j.name == "ECI":
                    y_j[12] = 1


                pos_i = np.concatenate((constraint.body_i.pos_v, constraint.body_i.pos_omega))
                pos_j = np.concatenate((constraint.body_j.pos_v, constraint.body_j.pos_omega))

                #Splits the state vectors for each body
                y_i[constraint.body_i.pos_v0] = y[constraint.body_i.pos_v]
                y_i[constraint.body_i.pos_omega0] = y[constraint.body_i.pos_omega]
                y_i[constraint.body_i.pos_r0] = y[constraint.body_i.pos_r]
                y_i[constraint.body_i.pos_Theta0] = y[constraint.body_i.pos_Theta]

                y_j[constraint.body_j.pos_v0] = y[constraint.body_j.pos_v]
                y_j[constraint.body_j.pos_omega0] = y[constraint.body_j.pos_omega]
                y_j[constraint.body_j.pos_r0] = y[constraint.body_j.pos_r]
                y_j[constraint.body_j.pos_Theta0] = y[constraint.body_j.pos_Theta]

                #Allocates the constraint vectors and matrices in the generalized mass matrix
                Cq_i, Cq_j, Cqf = constraint.C_matrices(y_i, y_j)
                if len(pos_i) > 0:
                    M[pos_c:pos_c + constraint.n_states, pos_i] = Cq_i
                    M[pos_i, pos_c:pos_c + constraint.n_states] = np.transpose(Cq_i)
                if len(pos_j) > 0:
                    M[pos_c:pos_c + constraint.n_states, pos_j] = Cq_j
                    M[pos_j, pos_c:pos_c + constraint.n_states] = np.transpose(Cq_j)

                #Alocates the terms associated with the constraint equations
                Q[pos_c:pos_c + constraint.n_states] = Cqf
                
                #Updates the position of the constriants in the generalized matrix
                pos_c += constraint.n_states
            
            M_temp = M
            #Solve for the acceleration and lagrange multipliers
            Y = np.matmul(np.linalg.inv(M), Q)            
            #Gather just the accelerations
            dy[0:system.DOF] = Y[0:system.DOF]

        return dy
    
                
                
                
                
                
        
            
    
        
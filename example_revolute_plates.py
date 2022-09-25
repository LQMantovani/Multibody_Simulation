import numpy as np
from base_functions import *
from classes_multibody import *
from plot_data import *

#This example illustrates 5 bodies connected to each other using revolute joints. The revolute joints orientation are define allowing them to rotate around the bodyies' x-axis. 

def external_forces(system,  t, y):
    #function that generates the external forces acting on the system
    U = np.zeros(system.DOF)
    for body in system.bodies:  
        if body.name == "Body_1":
            #Applies a force for a small amount of time to the system
            if t < 0.2:
                F_body = np.array([0, 0, 5])
            elif t < 0.4:
                F_body = np.array([0, 0, -5])
            else:
                F_body = np.array([0, 0, 0])
            U[body.pos_v] = np.matmul(np.transpose(quat2rot(y[body.pos_Theta])), F_body) 
        
    return U


def control_forces(system,  t, y):
    #Funtion that generates the control forces
    #This case defines springs on the revolute joints
    U = np.zeros(system.DOF)
    for cst in system.constraints:
        if isinstance(cst, Constraint_Revolute_Joint):

            a_j = cst.constraint_2_theta.a_j
            b_j = cst.constraint_2_theta.b_j
            c_j = cst.constraint_2_theta.c_j
            a_i = cst.constraint_2_theta.a_i

            AIj = np.transpose(quat2rot(y[cst.body_j.pos_Theta]))
            AIi = np.transpose(quat2rot(y[cst.body_i.pos_Theta]))

            angle = angle_joints(a_j, b_j, c_j, a_i, AIj, AIi)

            k_s = -0.1

            #Spring torque
            T_s = -angle*k_s

            factor = 1
            if np.dot(np.transpose(a_i), np.matmul(AIi, np.matmul(np.transpose(AIj), a_j))) < 0:
                factor = -1

            U[cst.body_j.pos_omega] += T_s*a_j.flatten()
            U[cst.body_i.pos_omega] += -T_s*a_i.flatten()*factor
                   
    return U



#Five bodies are created
body_1 = Body_rigid("Body_1", 10, np.identity(3))
body_2 = Body_rigid("Body_2", 10, np.identity(3))
body_3 = Body_rigid("Body_3", 10, np.identity(3))
body_4 = Body_rigid("Body_4", 10, np.identity(3))
body_5 = Body_rigid("Body_5", 10, np.identity(3))


#The bodies initial conditions are defined here,v_0, omega_0, x_0, Theta_0
body_1.set_X_0(np.array([0.1,0,0]), np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0,1]))
body_2.set_X_0(np.array([0.1,0,0]), np.array([0,0,0]), np.array([0,2,0]), np.array([0,0,0,1]))
body_3.set_X_0(np.array([0.1,0,0]), np.array([0,0,0]), np.array([0,-2,0]), np.array([0,0,0,1]))
body_4.set_X_0(np.array([0.1,0,0]), np.array([0,0,0]), np.array([0,4,0]), np.array([0,0,0,1]))
body_5.set_X_0(np.array([0.1,0,0]), np.array([0,0,0]), np.array([0,-4,0]), np.array([0,0,0,1]))

#A system is created named "car". A system class has body objects and constraint objects that express relations between the bodies.
car = System("car")

#The bodies are added to the system
car.add_body(body_1)
car.add_body(body_2)
car.add_body(body_3)
car.add_body(body_4)
car.add_body(body_5)


#Constraints are created and added to the system. Vectors d12, d21, d13, and d31 are written in the body's reference frame.
d12 = np.array([0, 1, 0])
d21 = np.array([0, -1, 0])

d13 = np.array([0, -1, 0])
d31 = np.array([0, 1, 0])

#A revolute joint constraint is used to constraint three translatinal degrees of freedom and two rotational degrees of freedom. It could also be achieved combning a "Constraint_r" and a "Constraint_2_theta" constraints (it actually call these constraints inside). Its advantage over the separate constraints is simplicity 
c_12 = Constraint_Revolute_Joint(body_1, body_2, d12, d21, np.array([1,0,0]), np.array([1,0,0]), 200, 0)
c_13 = Constraint_Revolute_Joint(body_1, body_3, d13, d31, np.array([1,0,0]), np.array([1,0,0]), 200, 0)
c_24 = Constraint_Revolute_Joint(body_2, body_4, d12, d21, np.array([1,0,0]), np.array([1,0,0]), 200, 0)
c_35 = Constraint_Revolute_Joint(body_3, body_5, d13, d31, np.array([1,0,0]), np.array([1,0,0]), 200, 0)

#Constraints are added to the system
car.add_constraint(c_12)
car.add_constraint(c_13)
car.add_constraint(c_24)
car.add_constraint(c_35)

#A World object is created, which supports  more than a system (actually it will support in the future. For now it supports only one system).
My_World = World("My world")

#The system is added to the world
My_World.add_system(car)

#A function that represents the control system is passed to the object (it is not required to simulate the system)
car.Set_Control(control_forces)
#A function that represetns the external forces is passed to the object (it is not required to simulate the sytsem)
car.Set_External(external_forces)

#The system is simulatd for 20 seconds in steps of 0.01 seconds
t, x = My_World.Simulate(0, 20, 0.01)

#The results are plotted
plot_data(My_World, t, x)
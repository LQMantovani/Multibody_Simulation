import numpy as np
from base_functions import *
from classes_multibody import *
from plot_data import *

#This example illustrates a pendulum that rotates about the x-axis in the inertial system. The cable length is 1m. It is initially parallel to the y-axis.

def external_forces(system,  t, y):
    #function that generates the external forces acting on the system
    g = 9.80665
    U = np.zeros(system.DOF)
    for body in system.bodies:  
        U[body.pos_v] = U[body.pos_v] + np.array([0,0,-body.mass*g])
        
    return U

#Two bodies are created with mass=10kg and an identity inertia matrix 
body_1 = Body_rigid("Body_1", 10, np.identity(3))
body_2 = Body_rigid("Body_2", 10, np.identity(3))

#The initial conditions of this body in the inertial system are defined
body_1.set_X_0(np.array([0,0,0]), np.array([0,0,0]), np.array([0,-1,0]), np.array([0,0,0,1]))
body_2.set_X_0(np.array([0,0,0]), np.array([0,0,0]), np.array([0,-2,0]), np.array([0,0,0,1]))

#A system is created. A system has bodies and constraints
car = System("car")

#Bodies are added to the system
car.add_body(body_1)
car.add_body(body_2)


d10 = np.array([0, 1, 0])
d01 = np.array([0, 0, 0])
d12 = np.array([0, -0.5, 0])
d21 = np.array([0, 0.5, 0])
#A constraint is created. This constraint allows for rotation around the x-axis and constraints three translational degrees of freedom
#The second body is set to "ECI", which corresponds to the Earth Centered Inertial Frame (it is an inertial frame)
c_10 = Constraint_Revolute_Joint(body_1, "ECI", d10, d01, np.array([1,0,0]), np.array([1,0,0]), 200, 0)
c_12 = Constraint_Revolute_Joint(body_1, body_2, d12, d21, np.array([1,0,0]), np.array([1,0,0]), 200, 0)
                

#The contraint is added to the system
car.add_constraint(c_10)
car.add_constraint(c_12)

#The World is created. A World is able to run the simulation and will accept more than one system at a time in the future
My_World = World("My world")

#The system is added to the world
My_World.add_system(car)

#The external forces are defined
car.Set_External(external_forces)

#Performs the simulation
t, x = My_World.Simulate(0, 20, 0.01)

#PLots the simulation
plot_data(My_World, t, x)
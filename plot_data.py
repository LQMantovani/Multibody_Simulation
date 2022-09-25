import numpy as np
from classes_multibody import *
from base_functions import *
import matplotlib.pyplot as plt

def plot_data(World, t, x):
    #Function to plot the states of the bodies and the constraints in the system
    #INPUTS:
    #World: class World
    #t: time vector
    #x: states vector
    
    fig = plt.figure(figsize = (10,10))
    ax = fig.add_subplot(111, projection = '3d')
    
    for system in World.systems:
        for body in system.bodies:
            
            if body.N_state[0] > 0:
                x_body = x[:,body.pos_r]

                ax.plot(x_body[:,0], x_body[:,1], x_body[:,2], label = body.name)


                ax.set_xlabel(' x (m) ')
                ax.set_ylabel(' y (m) ')
                ax.set_zlabel(' z (m) ')
            
    plt.show
    ax.legend()
    
    
    for system in World.systems:
        for body in system.bodies:  
            
            if body.N_state[0] > 0:
                v_body = x[:, body.pos_r]

                plt.figure(figsize = (10,10))
                plt.subplot(3,1,1)
                plt.plot(t, v_body[:,0])

                plt.xlabel(' x (m) ')
                plt.ylabel(' t (s) ')
                plt.title("Body " + body.name)

                plt.subplot(3,1,2)
                plt.plot(t, v_body[:,1])

                plt.xlabel(' y (m) ')
                plt.ylabel(' t (s) ')

                plt.subplot(3,1,3)
                plt.plot(t, v_body[:,2])

                plt.xlabel(' z (m) ')
                plt.ylabel(' t (s) ')
    
    
    for system in World.systems:
        for body in system.bodies:  
            
            if body.N_state[0] > 0:
                v_body = x[:, body.pos_v]

                plt.figure(figsize = (10,10))
                plt.subplot(3,1,1)
                plt.plot(t, v_body[:,0])

                plt.xlabel(' v_x (m/s) ')
                plt.ylabel(' t (s) ')
                plt.title("Body " + body.name)

                plt.subplot(3,1,2)
                plt.plot(t, v_body[:,1])

                plt.xlabel(' v_y (m/s) ')
                plt.ylabel(' t (s) ')

                plt.subplot(3,1,3)
                plt.plot(t, v_body[:,2])

                plt.xlabel(' v_z (m/s) ')
                plt.ylabel(' t (s) ')
            
            
    for system in World.systems:
        for body in system.bodies:  
            
            if body.N_state[1] > 0:
                state_b = x[:, body.pos_omega]

                plt.figure(figsize = (10,10))
                plt.subplot(3,1,1)
                plt.plot(t, state_b[:,0])

                plt.xlabel(' omega_x (rad/s) ')
                plt.ylabel(' t (s) ')
                plt.title("Body " + body.name)

                plt.subplot(3,1,2)
                plt.plot(t, state_b[:,1])

                plt.xlabel(' omega_y (rad/s) ')
                plt.ylabel(' t (s) ')

                plt.subplot(3,1,3)
                plt.plot(t, state_b[:,2])

                plt.xlabel(' omega_z (rad/s) ')
                plt.ylabel(' t (s) ')
            
            
    for system in World.systems:
        for body in system.bodies:  
            
            if body.N_state[1] > 0:
                quat = x[:, body.pos_Theta]

                state_b = np.zeros([quat.shape[0], 3])

                for i in range(0, quat.shape[0]):
                    state_b[i, :] = quat2angle(quat[i,:])

                plt.figure(figsize = (10,10))
                plt.subplot(3,1,1)
                plt.plot(t, state_b[:,0]*180/3.14159265358979)

                plt.xlabel(' phi (degree) ')
                plt.ylabel(' t (s) ')
                plt.title("Body " + body.name)

                plt.subplot(3,1,2)
                plt.plot(t, state_b[:,1]*180/3.14159265358979)

                plt.xlabel(' theta (degree) ')
                plt.ylabel(' t (s) ')

                plt.subplot(3,1,3)
                plt.plot(t, state_b[:,2]*180/3.14159265358979)

                plt.xlabel(' psi (degree) ')
                plt.ylabel(' t (s) ')
            
            
    for system in World.systems:
        for constraint in system.constraints:
            temp_C = np.zeros([len(t), constraint.n_states])
            for i in range(0, len(t)):
                y = x[i,:]

                y_i = np.zeros(len(constraint.body_i.X_0))
                y_j = np.zeros(len(constraint.body_j.X_0))
                if constraint.body_j.name == "ECI":
                    y_j[12] = 1

                y_i[constraint.body_i.pos_v0] = y[constraint.body_i.pos_v]
                y_i[constraint.body_i.pos_omega0] = y[constraint.body_i.pos_omega]
                y_i[constraint.body_i.pos_r0] = y[constraint.body_i.pos_r]
                y_i[constraint.body_i.pos_Theta0] = y[constraint.body_i.pos_Theta]

                y_j[constraint.body_j.pos_v0] = y[constraint.body_j.pos_v]
                y_j[constraint.body_j.pos_omega0] = y[constraint.body_j.pos_omega]
                y_j[constraint.body_j.pos_r0] = y[constraint.body_j.pos_r]
                y_j[constraint.body_j.pos_Theta0] = y[constraint.body_j.pos_Theta]  

                temp_C[i, :] = constraint.C_vector(y_i, y_j)

            plt.figure(figsize = (10,10))

            for i in range(1, constraint.n_states + 1):
                if constraint.n_states < 4:
                    plt.subplot(3,1,i)
                    plt.plot(t, temp_C[:, i - 1])
                elif constraint.n_states < 7:
                    plt.subplot(3,2,i)
                    plt.plot(t, temp_C[:, i - 1])
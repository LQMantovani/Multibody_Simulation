# Multibody_Simulation
This project aims to facilitate small multibody simulations of rigid bodies with certain constraints. 

File classes_multibody.py contain body, constraint, system, and world classes used to simulate multibody systems in a world environment. Basic functions, such as rotation matrices and skew-symmetric matrices are available in file basic_functions.py. File plot_data.py contains a function to plot position, velocity, angular velocity and orientation of the bodies, as well as the constriant values over time. 

Some use examples are given:
 - example_pendulum.py: This file contains the case of a pendulum composed of a single body constrained to move in a YZ plane.
 - example_double_pendulum.py: This case considers two bodies forming a double pendulum also constrained to move in the YZ plane.
 - example_revolute_plates.py: This case illustrate five bodies connected through revolute joints, with spring on these joints.

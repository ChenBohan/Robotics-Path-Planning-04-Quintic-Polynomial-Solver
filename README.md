# Robotics-Path-Planning-05-Quintic-Polynomial-Solver
Udacity Self-Driving Car Engineer Nanodegree: Quintic Polynomial Solver.

## Definition

Calculate the Jerk Minimizing Trajectory that connects the initial state to the final state in time T.

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-05-Quintic-Polynomial-Solver/blob/master/readme_img/Jerk%20Minimizing%20Trajectories.png" width = "70%" height = "70%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-05-Quintic-Polynomial-Solver/blob/master/readme_img/Polynomial%20solver.png" width = "70%" height = "70%" div align=center />

INPUTS:
- ``start``: The vehicles start location given as a length three array corresponding to initial values of [s, s_dot, s_double_dot].
- ``end``: The desired end state for vehicle. Like "start" this is a length three array.
- ``T``: The duration, in seconds, over which this maneuver should occur.

OUTPUT:
- an array of length 6, each value corresponding to a coefficent in the polynomial s(t) = a_0 + a_1 * t + a_2 * t ** 2 + a_3 * t ** 3 + a_4 * t ** 4 + a_5 * t ** 5

The equations for position, velocity, and acceleration are given by:

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-05-Quintic-Polynomial-Solver/blob/master/readme_img/formula1.png" width = "40%" height = "40%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-05-Quintic-Polynomial-Solver/blob/master/readme_img/formula2.png" width = "50%" height = "50%" div align=center />




PS: We are solving a system of equations: matrices will be helpful! The Eigen library used from Sensor Fusion is included.

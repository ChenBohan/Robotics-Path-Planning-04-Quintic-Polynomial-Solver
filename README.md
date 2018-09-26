# Robotics-Path-Planning-04-Quintic-Polynomial-Solver
Udacity Self-Driving Car Engineer Nanodegree: Quintic Polynomial Solver. 
& Paper 'Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame'

## Implement the Quintic Polynomial Solver

Calculate the Jerk Minimizing Trajectory that connects the initial state to the final state in time T.

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-05-Quintic-Polynomial-Solver/blob/master/readme_img/Jerk%20Minimizing%20Trajectories.png" width = "70%" height = "70%" div align=center />

Our problem is to find a function s of t that minimize the total jerk.

We find that all the time derivatives of s are further six and more have to be zero in order for s to be jerk minimize.

All minimum jerk trajectories can be repersented as a fifth order polynomial like that.

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

```cpp
vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;
		
	MatrixXd B = MatrixXd(3,1);	    
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];
			    
	MatrixXd Ai = A.inverse();
	MatrixXd C = Ai*B;
	
	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}
    return result;
}
```

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-05-Quintic-Polynomial-Solver/blob/master/readme_img/feasibility.png" width = "70%" height = "70%" div align=center />

## Paper 'Optimal Trajectory Generation for Dynamic Street Scenarios in a Frene´t Frame'

- The paper discusses some topics like:
	- Cost Functions.
	- Differences between high speed and low speed trajectory generation.
	- Implementation of specific maneuvers relevant to highway driving like following, merging, and velocity keeping.
	- How to combining lateral and longitudinal trajectories.
	- A derivation of the transformation from Frenet coordinates to global coordinates (in the appendix).

- Abstract
	- semi-reactive trajectory generation method
	- be tightly integrated into the behavioral layer
	- realize long-term objectives (such as velocity keeping, merging, following, stopping)
	- combine with a reactive collision avoidance
	- Frenét-Frame

- Related work
	-  [11], [19], [2], [4]: fail to model the inherent unpredictability of other traffic, and the resulting uncertainty, given that they **rely on precise prediction** of other traffic participant’s motions over a long time period.
	- [16], [1], [7]: The trajectories are represented parametrically. A finite set of trajectories is computed, typically by forward integration of the differential equations that describe vehicle dynamics.While this reduces the solution space and allows for fast planning, it may introduce **suboptimality**.
	- [9]: a tree of trajectories is sampled by simulating the closed loop system using the **rapidly exploring random tree algorithm** [10].
	- [17]: in a similar spirit to our method but only considers the free problem that is **not constrained by obstacle**.
	- We propose a **local method**, which is capable of realizing **high-level decisions** made by an upstream, **behavioral layer** (long-term objectives) and also **performs (reactive) emergency obstacle avoidance** in unexpected critical situations.
	
- Optimal control approach
	- system inputs or curvature to be **polynomials**.
	- cost functional is compliance with **Bellman’s principle** of optimality.
	- making the best compromise between the **jerk** and the **time**.
	- not limited to a certain function class, the problem becomes highly **complicated** and can be solved numerically at best.
	
<img src="https://github.com/ChenBohan/Robotics-Path-Planning-05-Quintic-Polynomial-Solver/blob/master/readme_img/overshoot.png" width = "50%" height = "50%" div align=center />

- Motion planning in the Frenet Frame

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-05-Quintic-Polynomial-Solver/blob/master/readme_img/fenet.png" width = "50%" height = "50%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-05-Quintic-Polynomial-Solver/blob/master/readme_img/formula3.png" width = "40%" height = "40%" div align=center />

Total jerk:
<img src="https://github.com/ChenBohan/Robotics-Path-Planning-05-Quintic-Polynomial-Solver/blob/master/readme_img/jerk1.png" width = "20%" height = "20%" div align=center />

Cost function:
<img src="https://github.com/ChenBohan/Robotics-Path-Planning-05-Quintic-Polynomial-Solver/blob/master/readme_img/cost1.png" width = "20%" height = "20%" div align=center />

A quintic polynomial through the same points and the same time interval will always lead to a smaller cost.

- Generation of lateral movement
	- High speed trajectories
		- at high speed, d(t) and s(t) can be chosen independently.
		- cost function: g(T)=T, h(d1)=d1^2.
		- process:
			1. calculate its coefficients and T minimizing.
			2. check it against collision.
			3. if not, check and find the second best and collision-free trajectory.
	- Low speed trajectories
		- at low speed, we must consider the non-holonomic property (invalid curvatures) of the car.
		- cost function: see in the paper.
- Generation of longitudianal movement
	- Following
		- safe distance (constant time gap law)
	- Merging
	- Stopping
	- Velocity keeping
- Combining lateral and longitudinal trajectories
	- check aginst outsized acceleration values first.
	- derive higher order infomation (heading, curvature, velocity, acceleration)
	- calculate the conjoint cost: Cost_total = w_lat * Cost_lat + w_lon * Cost_lon
	- for collison dectection: we add a certain safety distance to the size of our car on each side.

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-04-Quintic-Polynomial-Solver/blob/master/readme_img/paper_pic1.png" width = "50%" height = "50%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-04-Quintic-Polynomial-Solver/blob/master/readme_img/paper_pic2.png" width = "50%" height = "50%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-04-Quintic-Polynomial-Solver/blob/master/readme_img/paper_pic3.png" width = "50%" height = "50%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-04-Quintic-Polynomial-Solver/blob/master/readme_img/paper_pic_result.png" width = "50%" height = "50%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-05-Quintic-Polynomial-Solver/blob/master/readme_img/animation.gif" width = "70%" height = "70%" div align=center />

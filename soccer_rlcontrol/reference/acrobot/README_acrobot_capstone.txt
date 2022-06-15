TO RUN THE CODE:

- Run acrobot_main.m
- If running for the first time (or after making modifications to the model or VHC parameters), set run_from_scratch=1
- Running from scratch computes all necessary quantities (robot model matrices, impact map, virtual constraint splines, controller)
- If running the code from scratch, verify conditions for existence and stability of limit cycle:
	- existence_test < 0
	- 0 < stability_test < 1
- Will produce plots of:
	- Virtual constraint in Q space
	- Mass and Potential of reduced dynamics with respect to parameterizing variable theta
	- Check for regularity of VHC curve (should never cross 0)
	- q2_{desired} - q2 over the simulation time
	  (Should approach 0 within the first step and remain close to 0 for all remaining steps)

MODIFYING PARAMETERS:
***NOTE: modifying the model/vhc parameters may result in a constraint which does not meet conditions for existence and stability of
a limit cycle; in this case the acrobot will likely not be able to walk***
	- Robot model parameters: load_acrobot_params.m
		- m1 = stance leg mass, mH = hip mass (set to 0), m2 = swing leg mass
		- lc1 = stance foot to stance leg COM; l1 = stance foot to hip;
		  lc2 = hip to swing leg COM; l2 = hip to swing foot
	- Simulation parameters: acrobot_main.m
		- tmax: how long to run walking simulation; t_step = simulation step size
		- gamma, Kp, Kd = parameters for PD component of feedback linearization (affects convergence to constraint manifold)
		- x0 = [q1, q2, q1_dot, q2_dot] (Note: Must have high enough velocity to prevent rollback; in terms of the reduced dynamics,
			theta_dot must be close to the fixed point theta_dot_bar or higher to be in the domain of attraction/achieve forward walking)
	- Animation parameters: acrobt_main.m
		- scrolling: set to 1 for the plot window to move once the acrobot has reached the end of the current window
	- VHC parameters: compute_h_line.m
		- beta: desired leg aperture on impact; used to calculate q_p, q_m
			-q_p: post impact configuration
			-q_m: pre impact configuration
		- v, w: velocity vectors at the beginning/end of the constraint curve
		- v_w_scale: (leave this at 1)
		- kappa: angle of pre-impact slope (slope of w)
		- r0, r1, t0, t1: length of the line segments used to construct h(q)
		- r0_samples, ..., t1_samples: number of samples for each segment
		  (note: the lines extend slightly before q_p, and slightly after q_m)
		- r1_truncate, ..., t0_truncate: number of samples to truncate when performing the spline

OVERVIEW OF CODE:

	1. acrobot_symbolic.m:
		- Computes the equations of motion for the acrobot (matrices D, C, G), saves to file
		- Computes the impact map matrices
		- Saves parameters to acrobot_syms.mat
		- runs compute_h_line
	2. compute_h_line.m
		- Recomputes impact map (redundant)
		- Creates the points to spline to create the curve q = sigma(theta) with desired beginning and end slopes (slopes of v,w)
		- Computes the reduced dynamics and produces numbers to check for existence/stability of limit cycle
		- Computes reduced dynamics quatities: Psi1, Psi2, M, V
		- Produced plot for regularity check
		- Simulates reduced dynamics
		- Commented out: orbits over multiple steps, checking energy across solutions (should be const)
		- Saves constraint quantities to h_vars.mat, h_extra_vars.mat
	3. acrobot_main.m
		- Simulates the first step until impact, detected by impact_event.m
		- Until the end of simulation, at every impact:
			- Uses impact map to compute post-impact configuration/velocities (x0_new)
			- Reinitializes simulation with x0_new
			- Stores the "offset" (position of stance foot over the step)
		- Simulates acrobot walking (animate_acrobot_VHC)
	4. animate_acrobot_VHC:
		- left plot: acrobot walking
		- right plot: progression along the VHC (position in Q-space)

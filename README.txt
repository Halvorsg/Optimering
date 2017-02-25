All functions that are not mentioned here are helpfunctions used by these functions:
******
robot_BFGS_nudge.m takes in (p,L,tol,max_iter,thetaNudge,nudgeNumber):
- p = reaching point as [x,y]
- L = arm lengths as [L1; L2; L3]
- tol = tolerance for how low gradient must be to quit iterating
- max_iter = maximum of iterations
- thetaNudge = is 0 if called first time, is theta as [t1;t2;t3] if called again inside function.
- nudgeNumber = is 0 if called first time, used to stop if nudge does not work to get out of saddle point.

Example of call if one want to test the noise vector to get out of a saddle point:
(same as in text, but with another first angle and position to reach)
robot_BFGS_nudge([1.1,0],[4,2,1]',0.001,10000,[0,0,pi]',0)
******
robot_gradeint_descent.m takes in (p,L, tol, max_iter):
- Parameters are same as in robot_BFGS_nudge.m
******
convergencePlot.m runs without input parameters.
Uses robot_BFGS_nudge2.m which is just a simple BFGS and robot_gradeint_descent to make convergence plots

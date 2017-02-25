function [theta,n] = robot_gradeint_descent(p,L, tol, max_iter)
%% Preconditions 
%p - point desired
%L - Column ector of length segments
%tol - Toleration for the norm of the gradient
%max_iter - Max iterations
% Do not call this function with max_iter less than 1000 if you want to
% plot the solution.
%% Postconditions
%theta - Column vector of the angle between the length segments
%n - Number of iterations
%% Case 1 - point out of reach
tic
dist_to_p = norm(p);
RADIUS = sum(L)-tol; %Outer radius
[MAX,place] = max(L);
radius = MAX-(RADIUS-MAX);%Inner radius
if dist_to_p >= RADIUS % If p in outer disc
    theta = is_outside(L,p);
    n = 0;
    toc
    robot_arm(theta,L,p);
    return
elseif dist_to_p <=radius %if p in inner disc
    theta = is_inside(L,p,place);
    n = 0;
    toc
    robot_arm(theta,L,p);
    return
end

%% Case 2 - point within reach
theta = ones(length(L),1).*pi/2;
%Dette er de to funksjonene
d = @(theta,L,p) 1/2*norm([sum(L.*cos(cumsum(theta))),sum(L.*sin(cumsum(theta)))]-p)^2;
dd = robot_gradient(theta,L,p);

alpha0 = 1;
rho = 1/2;
c = 1/4;
n = 0;
while norm(dd) > tol && n<=max_iter
    pk = -dd; %Gradient descent
    alpha = alpha0;
    c_dd_dot_pk = c*dot(dd,pk);
    
    while d(theta+alpha*pk,L,p) > d(theta,L,p)+alpha*c_dd_dot_pk %Armijo condition
        alpha = rho*alpha;
    end
    theta = theta + alpha * pk;
    dd = robot_gradient(theta,L,p);
    n = n+1;
end
toc
%% For the numerical experiment
if max_iter>=1000
    robot_arm(theta,L,p);
end
end
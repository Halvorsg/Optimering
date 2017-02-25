function [theta,n] = robot_gradeint_descent(p,L, tol, max_iter)
% Do not call this function with max_iter less than 1000 if you want to
% plot the solution.

theta0 = ones(length(L),1).*pi/2;
%Dette er de to funksjonene
tic
d = @(theta,L,p) 1/2*norm([sum(L.*cos(cumsum(theta))),sum(L.*sin(cumsum(theta)))]-p)^2;
dd = robot_gradient(theta0,L,p);

alpha0 = 1;
rho = 1/2;
c = 1/4;
theta = theta0;
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
if max_iter>=1000
    robot_arm(theta,L,p);
end
end
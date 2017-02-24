function [theta,n] = robot_BFGS_trust(p,L,tol)
delta = 0.5;
nu = 1/4;
kappa = 3/4;
d = @(theta,L,p) 1/2*norm([sum(L.*cos(cumsum(theta))),sum(L.*sin(cumsum(theta)))]-p)^2;
dd = robot_gradient(theta,L,p);
m = @(f,g,B,p) f+g'*p+1/2*p'*B*p;
I = eye(length(L)).*10^-3;
H = I.*10^-3; %H er inv(B), ikke sant?
max_iter = 10000;
n = 0;
while norm(dd)<tol && n < max_iter
    pk = 1; %solved approximately for min(mk)
    rok = (d(theta)-d(theta+pk))/(d(theta)-m(d(theta),dd,inv(H),pk));
    if rok< 1/4
        delta = 1/4*delta;
    else
        if rok > 3/4 && norm(pk) == delta
        











end
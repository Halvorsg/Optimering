function [theta,n] = robot_BFGS_nudge(p,L,tol,max_iter,thetaNudge,nudgeNumber)
%% Preconditions
%p - desired point
%L - Column ector of length segments
%tol - Toleration for the norm of the gradient
%max_iter - Max iterations
%thetaNudge - 
%nudgeNumber - 
%% Post conditions 
%theta - the angels between each segment
%n - number of iterations
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
if thetaNudge == 0
    theta = ones(length(L),1);
else
    theta = thetaNudge;
end
% d is the function we want to minimize
% dd is the gradient of that function
d = @(theta,L,p) 1/2*norm([sum(L.*cos(cumsum(theta))),sum(L.*sin(cumsum(theta)))]-p)^2;
dd = robot_gradient(theta,L,p);
%Initialiazing values
I = eye(length(L))*0.01;
H = I;
n = 0;
while norm(dd) > tol && n<=max_iter
    n = n+1;
    %Initial step direction
    pk = -H*dd;
    %finding step length
    alpha = find_alpha(pk,theta,L,p);
    %Updating theta
    theta = theta + alpha*pk;
    %Updating H
    sk = alpha*pk;
    yk = (robot_gradient(theta+alpha*pk,L,p)-dd);
    rok = 1/dot(yk,sk);
    zk = (H*yk);
    H = H - rok*(sk*zk' + zk*sk') + (rok^2*dot(yk,zk)+rok)*(sk*sk');
    %Updating dd
    dd = robot_gradient(theta,L,p);
end 
toc
%% Checking if theta is a saddle point
distToTarget = d(theta,L,p);
distTolerance = 10^-3;
if (norm(p)<=sum(L) && norm(p)>=2*max(L)-sum(L) && distToTarget > distTolerance)%point inside C
    %if all are true, then we are 100% certain that we are in a saddle point.
    if nudgeNumber == 5
        fprintf('We did not get out of the saddle point')
        return
    end
    for i = 1:length(theta)
        theta(i) = theta(i) + mod(rand(),0.2*nudgeNumber)-0.1*nudgeNumber; %bigger and bigger nudges
    end
    [theta,n] = robot_BFGS_nudge(p,L,tol,max_iter,theta, nudgeNumber+1);
end
if nudgeNumber == 0
    robot_arm(theta,L,p);
end
end
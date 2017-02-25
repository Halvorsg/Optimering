function [theta,n] = robot_BFGS_nudge(p,L,tol,thetaNudge,nudgeNumber)
tic
[flag1,theta] = is_outside(L,p);
if flag1 == true %If point outside of circle we may terminate
    n = 0;
    robot_arm(theta,L,p);
    toc
    return
end
[flag1,theta] = is_inside(L,p);
if flag1 == true %If point inside of circle we may terminate
    n = 0;
    %robot_arm(theta,L,p);
    toc
    return
end

if thetaNudge == 0
    theta = ones(length(L),1);
else
    theta = thetaNudge;
end
%Dette er de to funksjonene
d = @(theta,L,p) 1/2*norm([sum(L.*cos(cumsum(theta))),sum(L.*sin(cumsum(theta)))]-p)^2;
dd = robot_gradient(theta,L,p);
%Initialiazing values
I = eye(length(L))*0.01;
H = I;
% rho = 1/2;
% c1 = 1/10^4;
% c2 = 0.9;
% dtheta = d(theta,L,p);
max_iter = 100000;
n = 0;
while norm(dd) > tol && n<=max_iter
    n = n+1;
    %Initial step length and direction
%     alpha = 1;
    pk = -H*dd;
    %finding step length
%     c1_df_dot_pk = c1*dot(dd,pk);
%     c2_df_dot_pk = c1_df_dot_pk/c1*c2;
    
    alpha = find_alpha(pk,theta,L,p);
%     while (d(theta+alpha*pk,L,p) > dtheta+alpha*c1_df_dot_pk || robot_gradient(theta+alpha*pk,L,p)'*pk <= c2_df_dot_pk) && alpha>10^(-13) %Wolfe Conditions
%         %Blir det ikke feil ulikhet ved andre cond?
%         alpha = rho*alpha;
%     end
    %Updating x
    theta = theta + alpha*pk;
    %Updating H
    sk = alpha*pk;
    yk = (robot_gradient(theta+alpha*pk,L,p)-dd);
    rok = 1/dot(yk,sk);
    zk = (H*yk);
    H = H - rok*(sk*zk' + zk*sk') + (rok^2*dot(yk,zk)+rok)*(sk*sk');
    %Updating fx and dd
%     dtheta = d(theta,L,p);
    dd = robot_gradient(theta,L,p);
end 
toc
distToTarget = d(theta,L,p);
distTolerance = 10^-3;
if (norm(p)<=sum(L) && norm(p)>=2*max(L)-sum(L) && distToTarget > distTolerance)...%point inside C
        ||(norm(p)>sum(L) && distToTarget>norm(p)-sum(L)+distTolerance)||...%point to far out
        (norm(p)<2*max(L)-sum(L) && distToTarget> 2*max(L)-sum(L)-norm(p)+distTolerance)%point to close origo to reach
    %if all are true, then we are 100% certain that we are in a saddle point.
    if nudgeNumber == 5
        fprintf('We did not get out of the saddle point')
        return
    end
    for i = 1:length(theta)
        theta(i) = theta(i) + mod(rand(),0.2*nudgeNumber)-0.1*nudgeNumber; %bigger and bigger nudges
    end
    [theta,n] = robot_BFGS_nudge(p,L,tol,theta, nudgeNumber+1);
end
if nudgeNumber == 0
    %robot_arm(theta,L,p);
end
end
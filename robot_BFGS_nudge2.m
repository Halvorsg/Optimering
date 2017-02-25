function [theta,n] = robot_BFGS_nudge2(p,L,tol,max_iter,thetaNudge,nudgeNumber)
tic
[flag,theta] = is_outside(L,p);
if flag == true %If point outside of circle we may terminate
    n = 0;
    robot_arm(theta,L,p);
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


end
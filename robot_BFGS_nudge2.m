function [theta,n] = robot_BFGS_nudge2(p,L,tol,max_iter,thetaNudge,nudgeNumber)
tic

theta = ones(length(L),1);
%Dette er de to funksjonene
d = @(theta,L,p) 1/2*norm([sum(L.*cos(cumsum(theta))),sum(L.*sin(cumsum(theta)))]-p)^2;
dd = robot_gradient(theta,L,p);
%Initialiazing values
I = eye(length(L))*0.01;
H = I;
n = 0;
while norm(dd) > tol && n<=max_iter
    n = n+1;
    %Initial step length and direction
%     alpha = 1;
    pk = -H*dd;
    %finding step length
    
    alpha = find_alpha(pk,theta,L,p);
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
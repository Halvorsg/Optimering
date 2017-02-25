function [theta,n] = robot_BFGS_fun(p,L,tol)
tic
%[theta,n] = robot_gradeint_descent(p,L, tol, 5);
theta = ones(length(L),1);

dist_to_p = norm(p);
RADIUS = sum(L); %Outer radius
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





%Dette er de to funksjonene
d = @(theta,L,p) 1/2*norm([sum(L.*cos(cumsum(theta))),sum(L.*sin(cumsum(theta)))]-p)^2;
dd = robot_gradient(theta,L,p);
%Initialiazing values
I = eye(length(L));
H = I;
max_iter = 100000;
n = 0;
while norm(dd) > tol && n<=max_iter
    n = n+1;
    %Initial step length and direction
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
    dd = robot_gradient(theta,L,p);

end
toc
robot_arm(theta,L,p);
end
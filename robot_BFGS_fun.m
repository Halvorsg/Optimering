function [theta,n] = robot_BFGS_fun(p,L,tol)
tic
%[theta,n] = robot_gradeint_descent(p,L, tol, 5);
theta = ones(length(L),1);
%Dette er de to funksjonene
d = @(theta,L,p) 1/2*norm([sum(L.*cos(cumsum(theta))),sum(L.*sin(cumsum(theta)))]-p)^2;
dd = robot_gradient(theta,L,p);
%Initialiazing values
I = eye(length(L));
H = I;
rho = 1/2;
c1 = 1/10^4;
c2 = 0.9;
dtheta = d(theta,L,p);
max_iter = 100000;
n = 0;
while norm(dd) > tol && n<=max_iter
    n = n+1;
    %Initial step length and direction
    alpha = 1;
    pk = -H*dd;
    %finding step length
    c1_df_dot_pk = c1*dot(dd,pk);
    c2_df_dot_pk = c1_df_dot_pk/c1*c2;
    al = 0;
    ar = 1;
    alpha = zoom(pk,theta,L,c1,p);
    while d(theta+alpha*pk,L,p) > dtheta+alpha*c1_df_dot_pk || robot_gradient(theta+alpha*pk,L,p)'*pk >= c2_df_dot_pk %Wolfe Conditions
        %Blir det ikke feil ulikhet ved andre cond?
        alpha = rho*alpha;
    end
    %Updating x
    theta = theta + alpha*pk;
    %Updating H
    sk = alpha*pk;
    yk = (robot_gradient(theta+alpha*pk,L,p)-dd);
    rok = 1/dot(yk,sk);
    zk = (H*yk);
    H = H - rok*(sk*zk' + zk*sk') + (rok^2*dot(yk,zk)+rok)*(sk*sk');
    %Updating fx and dd
    dtheta = d(theta,L,p);
    dd = robot_gradient(theta,L,p);

end
toc
robot_arm(theta,L,p);
end
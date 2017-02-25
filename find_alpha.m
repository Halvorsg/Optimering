function alpha = find_alpha(pk,theta,L,p)
%% Preconditions
%pk - Step direction
%theta - vector with angels corresponding to the segments in L
%L - vector with lengths og each segment
%p - desired point
%% Postconditions 
%alpha - steplength fullfilling Wolfe condidions
%% Initiating values
d = @(theta,L,p) 1/2*norm([sum(L.*cos(cumsum(theta))),sum(L.*sin(cumsum(theta)))]-p)^2;
dd = robot_gradient(theta,L,p);
al = 0;
ar = 1;
c2 = 0.9;
c1 = 10^-4;
c1_dd_dot_pk = c1*dot(dd,pk);
c2_df_dot_pk = c1_dd_dot_pk/c1*c2;
alpha = 1;
%% Finding alpha
while ~(d(theta+alpha*pk,L,p) <= d(theta,L,p)+alpha*c1_dd_dot_pk && robot_gradient(theta+alpha*pk,L,p)'*pk >= c2_df_dot_pk)&& ar~=al
    %while Wolfe conditiones not satisfied 
    if ~(d(theta+alpha*pk,L,p) <= d(theta,L,p)+alpha*c1_dd_dot_pk) %~(2. Wolfe cond.)
        ar = alpha;%Decrease ar
    else
        al = alpha;%Increase al
    end
    alpha = (al+ar)/2;
end
end
    
function alpha = zoom(pk,theta,L,c,p)
d = @(theta,L,p) 1/2*norm([sum(L.*cos(cumsum(theta))),sum(L.*sin(cumsum(theta)))]-p)^2;
dd = robot_gradient(theta,L,p);
al = 0;
ar = 1;
c2 = 0.9;
c1 = 10^-4;
c1_dd_dot_pk = c1*dot(dd,pk);
c2_df_dot_pk = c1_dd_dot_pk/c1*c2;
alpha = (al+ar)/2;
while ~(d(theta+alpha*pk,L,p) <= d(theta,L,p)+alpha*c1_dd_dot_pk && robot_gradient(theta+alpha*pk,L,p)'*pk >= c2_df_dot_pk)
    if ~(d(theta+alpha*pk,L,p) <= d(theta,L,p)+alpha*c1_dd_dot_pk)
        ar = alpha;
    else
        al = alpha;
    end
    alpha = (al+ar)/2;
end
    
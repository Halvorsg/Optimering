function alpha = zoom(pk,theta,L,p)
%Finner alpha som stemmer med wolfe betingelsene 
%Sto p� arket han skrev at man skulle �ke ar til armijo holder, vet ikke om dette gj�re korrekt her. 
d = @(theta,L,p) 1/2*norm([sum(L.*cos(cumsum(theta))),sum(L.*sin(cumsum(theta)))]-p)^2;
dd = robot_gradient(theta,L,p);
al = 0;
ar = 1;
%Vil �ke ar til Armijo holder?
c2 = 0.9;
c1 = 10^-4;
c1_dd_dot_pk = c1*dot(dd,pk);
c2_df_dot_pk = c1_dd_dot_pk/c1*c2;
%burde v�re mulig � finne en bedre m�te � finne ar p�...
while ~(d(theta+ar*pk,L,p) <= d(theta,L,p)+ar*c1_dd_dot_pk)
    ar = ar+0.05;
end
alpha = (ar+al)/2;
while ~(d(theta+alpha*pk,L,p) <= d(theta,L,p)+alpha*c1_dd_dot_pk && robot_gradient(theta+alpha*pk,L,p)'*pk >= c2_df_dot_pk)
    %while Wolfe conditiones not satisfied 
    if ~(d(theta+alpha*pk,L,p) <= d(theta,L,p)+alpha*c1_dd_dot_pk) %~(2. Wolfe cond.)
        ar = alpha;%Decrease ar
    else
        al = alpha;%Increase al
    end
    alpha = (al+ar)/2;
end
end
    
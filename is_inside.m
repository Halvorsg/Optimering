function [flag,theta] = is_inside(L,p)

theta = zeros(length(L),1);
SUM = sum(L);
dist_to_p = norm(p);
[MAX,n] = max(L);
radius = MAX-(SUM-MAX);
    thetaS = atan(p(2)/p(1));
    
    if p(1)<0
        thetaS=thetaS+pi;     
    end

if dist_to_p <= radius
    flag = true;
    if n == 1
        theta(1) = thetaS;
        theta(2) = pi;
    elseif n ~= 1
        theta(1) = thetaS + pi;
        theta(n) = pi;
        theta(n+1) = pi;
    end
else
    flag = false;
end
end

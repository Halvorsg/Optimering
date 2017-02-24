function [flag,theta] = is_outside(L,p)
%Checks if the point p i out of reach for the robot arm
%return a flag indicating true if it is and false otherwise. Also returns
%theta 
theta = zeros(length(L),1);
len = sum(L);
dist_to_p = norm(p);
if len > dist_to_p
    flag = false;
else
    flag = true;
    theta(1) = atan(p(2)/p(1));   
end
end
    
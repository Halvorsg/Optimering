function theta = is_outside(L,p)
%Takes in L as a vector of the length of the segmenst and p as the desired
%point known to be out of reach for the robot arm

%Returns the angels to minimize the distance to the point p
%% Initiating theta
theta = zeros(length(L),1);
%% Finding the first angle
theta(1) = atan(p(2)/p(1));
%% If x-value negative
if p(1)<0 
    theta(1)=theta(1)+pi;     
end
end
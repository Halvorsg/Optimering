function theta = is_inside(L,p,n)
%% Initiating theta
theta = zeros(length(L),1);
%% Setting the first angle
if norm(p) ~= 0
    thetaS = atan(p(2)/p(1));
else
    thetaS = 0; %if atan(0/0)
end
%% if x-value negative    
    if p(1)<0 
        thetaS=thetaS+pi;     
    end
%% Setting the other angels
    if n == 1
        theta(1) = thetaS;
        theta(2) = pi;
    elseif n ~= 1
        theta(1) = thetaS + pi;
        theta(n) = pi;
        theta(n+1) = pi;
    end
end

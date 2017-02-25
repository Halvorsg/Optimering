function dtheta = robot_gradient(theta, L ,p)

THETA = cumsum(theta);
L_cos_theta = L.*cos(THETA);
L_sin_theta = L.*sin(THETA);
pos_arm = [sum(L_cos_theta),sum(L_sin_theta)];

X = flip(L_sin_theta);
X = -cumsum(X); %Flippes senere i dtheta

Y = flip(L_cos_theta);
Y = cumsum(Y); %Flippes senere i dtheta


dtheta = (flip(X.*(pos_arm(1)-p(1))+Y.*(pos_arm(2)-p(2))));

end
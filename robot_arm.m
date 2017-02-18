function final_pos = robot_arm(theta,L)
% Kan sende in theta = [30,30,30] og L = [1,1,1]
%Plotter en graf som visualiserer robotarmen
r = sum(L);
q = zeros(length(theta),2);
THETA = cumsum(theta);
for i = 1:length(theta)
    q(i+1,1:2) = q(i,1:2)+L(i).*[cos(THETA(i)),sin(THETA(i))];
end
plot(q(:,1),q(:,2),'-or')
xlim([-r,r]);
ylim([-r,r]); 
final_pos = [sum(L.*cos(THETA)),sum(L.*sin(THETA))];
end

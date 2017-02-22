function final_pos = robot_arm(theta,L,p)
% Kan sende in theta = [30,30,30] og L = [1,1,1]
%Plotter en graf som visualiserer robotarmen
r = sum(L);
q = zeros(length(theta),2);
THETA = cumsum(theta);
for i = 1:length(theta)
    q(i+1,1:2) = q(i,1:2)+L(i).*[cos(THETA(i)),sin(THETA(i))];
end
figure
plot(q(1,1),q(1,2),'*b')
xlim([min(-p(1),-r),max(p(1),r)]);
ylim([min(-p(2),-r),max(p(2),r)]); 
hold on;
plot(q(:,1),q(:,2),'-or')
plot(q(end,1),q(end,2),'*k')
plot(p(1),p(2),'Og')
legend('Start point','Arm','End point arm','Reaching point')
final_pos = [sum(L.*cos(THETA)),sum(L.*sin(THETA))];
end

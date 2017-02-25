function final_pos = robot_arm(theta,L,p)
r = sum(L);
q = zeros(length(theta),2);
THETA = cumsum(theta);
for i = 1:length(theta)
    q(i+1,1:2) = q(i,1:2)+L(i).*[cos(THETA(i)),sin(THETA(i))];
end
figure
plot(q(1,1),q(1,2),'*b')
xlim([min(-abs(p(1)),-r),max(p(1),r)]);
ylim([min(-abs(p(2)),-r),max(p(2),r)]); 
hold on;
plot(q(:,1),q(:,2),'-or')
plot(q(end,1),q(end,2),'*k')
plot(p(1),p(2),'Og')
legend('Start point','Arm','End point arm','Reaching point')
final_pos = [sum(L.*cos(THETA)),sum(L.*sin(THETA))];
end



iter=40;
tol=zeros(1,8);
x=zeros(1,8);
tol0=0.1;
convBFGS=zeros(1,8);
convGD=zeros(1,8);
distBFGS=zeros(1,iter);
distGD=zeros(1,iter);

p=[10,10];
L=[3,2,1,1,3,4,5,3,2,3,4,5,6,1,1]';


for i=1:8
    tol(i)=0.1*tol0^i;
    x(i)=1/tol(i);
end


for i=1:8
    [~,convBFGS(i)]=robot_BFGS_nudge2(p,L,tol(i),100000,0,0);
    [~,convGD(i)]=robot_gradeint_descent(p,L,tol(i),500);
end

for i=1:iter
    [theta,~]=robot_BFGS_nudge2(p,L,-1,i,0,0);
    distBFGS(i)=norm(p-robot_arm2(theta,L,p));
    [theta2,~]=robot_gradeint_descent(p,L,-1,i);
    distGD(i)=norm(p-robot_arm2(theta2,L,p));
end

disp(convBFGS(8))
disp(convGD(8))

iterate=1:iter;

figure(1)
plot(log10(tol),convBFGS)
set(gca,'Xdir','reverse')
hold on
plot(log10(tol),convGD)
xlabel('Log of tolerance')
ylabel('Number of iterations')
legend('Number of iterations for BFGS','Number of iterations for Gradient Descent')
title('Convergence plot for the two methods')

figure(2)
plot(iterate,log10(distBFGS))
hold on
plot(iterate,log10(distGD))
title('Log of distance to p vs number of iterations')
xlabel('Number of iterations')
ylabel('Log of distance to p')
legend('Log of distance to p for BFGS','Log of distance to p for Gradient Descent')

%le script for å sjekke konvergens av quasi-newton (BFGS), og gradient
%descent
tol=zeros(1,8);
x=zeros(1,8);
tol0=0.1;
convBFGS=zeros(1,8);
convGD=zeros(1,8);
p=[10,10];
L=[3,2,1,1,3,4,5,3,2,3,4,5,6,1,1,1]';


for i=1:8
    tol(i)=0.1*tol0^i;
    x(i)=1/tol(i);
end


for i=1:8
    [~,convBFGS(i)]=robot_BFGS_nudge(p,L,tol(i),0,0);
    [~,convGD(i)]=robot_gradeint_descent(p,L,tol(i),100000);
end

plot(log10(tol),convBFGS)
hold on
plot(log10(tol),convGD)
xlabel('Log of tolerance')
ylabel('Number of iterations')
disp(tol)
legend('# of iterations for BFGS','# of iterations for gradient descent')
axis([-9,-2,0,400])
disp(size(L))
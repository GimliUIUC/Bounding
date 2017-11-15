function [] = tempplot(t,X)
subplot(2,2,1)
plot(t,X(:,1:3))
legend('x','z','q1')

subplot(2,2,2)
plot(t,X(:,4:6))
legend('dx','dz','dq1')

subplot(2,2,3)
plot(t,X(:,7:8))
legend('s_b','s_f')

subplot(2,2,4)
plot(t,X(:,9:10))
legend('t','T')

end
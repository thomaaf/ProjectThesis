N = 5;
bound = 105000;
start = 99000;
theta = out.theta(start:bound,:);
nums = out.numsopt(start:bound,:);
xpos = out.x(start:bound,1:2);
V    = out.V(start:bound,1);
V_calc = zeros(start - bound,1);
L_sum  = zeros(start - bound,1);
P_sum  = zeros(start - bound,1);
for i = 1:size(xpos,1)
    A = [nums(i,35:36);nums(i,37:38)];
    B = [nums(i,40);nums(i,41)];
    E = [nums(i,42);nums(i,43)];
    Q = [nums(i,31:32);nums(i,33:34)];
    R =  nums(i,39);
    F =  nums(i,44:46)';
    P = [nums(i,47:48);nums(i,49:50)];
    x = [nums(i,1:6);nums(i,7:12)];
    u = [nums(i,13:18)];
    L_sum(i) = 0;
    for j = 1:N
       L_sum(i) = L_sum(i)+ 0.5*(RLParam.gamma^(j-1)*(x(:,j)'*Q*x(:,j) + R*u(j)^2)) + F'*[x(:,j);u(j)];
       F'*[x(:,j);u(j)];
    end
    P_sum(i) = 0.5*RLParam.gamma^(N+1)*x(:,N+1)'*P*x(:,N+1);
    V_calc(i) = L_sum(i) + P_sum(i);
end
figure(4)
clf(4)
plot(V_calc); hold on;
yyaxis right
plot(xpos(:,1))
legend('Vcalc','pos')
grid on
clear
%-----MPC Initial Values
n = 4;
N = 20; dt = 0.1; % Approx entire dynamics
A = [0,1,0,0;0,0,4.90500000000000,0;0,0,0,1;0,0,14.7150000000000,0];
B = [0;1;0;2];    
P = eye(4);
%-----MPC CASADI Setup------
opti = casadi.Opti();
opti_init = casadi.Opti()
X = opti_init.variable(n,N+1);
    CartPos = X(1,:);
    CartVel = X(2,:);
    PendPos = X(3,:);
    PendVel = X(4,:); 
U = opti_init.variable(1,N);
%Obj and eq Constraints
for i = 1:N
    xnext = X(:,i) + dt*(A*X(:,i) + B*U(i));
    opti_init.subject_to(X(:,i+1) == xnext) % Xn+1 = Ax + BU
    opti_init.minimize(X(:,i)'*P*X(:,i))    
end
opts = struct; opts.ipopt.print_level= 0; opts.verbose = 0;
opti.solver('ipopt',opts);

%-----SimulationInitialValues----
x0 = [1;0;0.4;0];
tspan = 12; h = 0.01;
%-----SimulationSetup------
t = [0;tspan];
x = zeros(t(2)/h,size(x0,1));
x(1,:) = x0;

%-----Simulation loop-------
for t = 1:tspan/h
    %LQR
    %u = -[-1.00000000000000,-3.25247081999382,19.1032170908098,6.08193031548694]*x(t,:)';
    %MPC
    opti.subject_to(); 
    
	opti.subject_to(X(1:n,1) == x(t,:)');
    sol = opti.solve();
    u = sol.value(U(1));   
%------RK4-------    
    x = RK4(x,u,h,t);
end

Analyze(x,0:h:12);

function x = RK4(x,u,dt,t)
    k1 = dt*Plant(x(t,:)',u);
    k2 = dt*Plant(x(t,:)' + k1/2,u);
    k3 = dt*Plant(x(t,:)' + k2/2,u);
    k4 = dt*Plant(x(t,:)' + k3,u);
    x(t+1,:) = x(t,:) + 1/6*(k1 + 2*k2 + 2*k3 + k4)';
end

 
% function dx = simulation(t,x)
%     P = eye(4); u1 = MPC(x,P); u2 =
%     -[-1.00000000000000,-3.25247081999382,19.1032170908098,6.08193031548694]*x(5:8);
%     dx1 = Plant(x(1:4),u1); dx2 = Plant(x(5:8),u2); dx = [dx1;dx2];
%     %u_tmp = evalin('base','u'); assignin('base','u',[u_tmp;u]) x
% end
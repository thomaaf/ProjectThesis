function u = MPC(x0,P)
    N = 20; dt = 0.1; % Approx entire dynamics
    A = [0,1,0,0;0,0,4.90500000000000,0;0,0,0,1;0,0,14.7150000000000,0];
    B = [0;1;0;2];    
    opti = casadi.Opti();
%----Optimization variables-----
    x = opti.variable(4,N+1);
        CartPos = x(1,:);
        CartVel = x(2,:);
        PendPos = x(3,:);
        PendVel = x(4,:); 
    
    u = opti.variable(1,N);
%-------Euler Progression, Objective Function, EQ Constraints----
    for i = 1:N
        xnext = x(:,i) + dt*(A*x(:,i) + B*u(i));
        opti.subject_to(x(:,i+1) == xnext) % Xn+1 = Ax + BU
        opti.minimize(x(:,i)'*P*x(:,i) )    
    end
%-------Initial conditions / feedback----------
    for i = 1:size(x,1)
        opti.subject_to(x(i,1) == x0(i));
    end
    opts = struct; opts.ipopt.print_level= 0; opts.verbose = 0;
    opti.solver('ipopt',opts);
    sol = opti.solve();
    u = sol.value(u(1));
    
    
end


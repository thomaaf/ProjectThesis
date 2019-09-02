function [x,u,t,uopt,xopt] = Simulation(A,B,P,N,dt,n,tspan,h,x0)
    %-----MPC CASADI Setup------
    opti = casadi.Opti();
    X = opti.variable(n,N+1);
        CartPos = X(1,:);
        CartVel = X(2,:);
        PendPos = X(3,:);
        PendVel = X(4,:); 
    U = opti.variable(1,N);
    %Obj and eq Constraints
    xnext = casadi.MX(4,N);
    obj = casadi.MX(1);
    for i = 1:N
        xnext(:,i) = X(:,i) + dt*(A*X(:,i) + B*U(i));
        obj = obj +  X(:,i)'*P*X(:,i);
    end
    opti.minimize(obj);
    opts = struct; %opts.ipopt.print_level= 0; opts.verbose = 0;
    opti.solver('ipopt',opts);

    %-----SimulationSetup------
    t = [0;tspan];
    x = zeros(t(2)/h,size(x0,1));
    u = zeros(t(2)/h,1);
    uopt = zeros(t(2)/h,N);
    xopt = zeros(t(2)/h,N+1);
    x(1,:) = x0;

    %-----Simulation loop-------
    for t = 1:tspan/h
    %LQR
        %u(t) = -[-2.000000000000016,-5.352043890191520,22.467581687680017,7.820687307165484]*x(t,:)';
    %MPC
        opti.subject_to(); %Clear prev constraints;
        opti.subject_to(X(:,2:end) == xnext)
        opti.subject_to(X(1:n,1) == x(t,:)');
        
        
        try 
            sol = opti.solve();
        catch ME
           mpause(1); 
        end
        u(t) = sol.value(U(1));   
        uopt(t,:) = sol.value(U);
        xopt(t,:) = sol.value(X(1,:));
%         figure(3);
%         subplot(2,2,1); plot(sol.value(X(1,:)))
%         subplot(2,2,2); plot(sol.value(X(2,:)))
%         subplot(2,2,3); plot(sol.value(U(1,:)))
%         subplot(2,2,4); plot(sol.value(X(4,:)))
%         pause(0.01);
    %------RK4-------    
        x = RK4(x,u(t),h,t);
        disp(t);
    end
    t = 0:h/tspan:tspan;
    Analyze(x,u,0:h:tspan);

end

function x = RK4(x,u,dt,t)
    k1 = dt*Plant(x(t,:)',u);
    k2 = dt*Plant(x(t,:)' + k1/2,u);
    k3 = dt*Plant(x(t,:)' + k2/2,u);
    k4 = dt*Plant(x(t,:)' + k3,u);
    x(t+1,:) = x(t,:) + 1/6*(k1 + 2*k2 + 2*k3 + k4)';
end

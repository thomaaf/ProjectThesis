function [x,u,xopt,uopt,t]= Simulation(A,B,Q,Qsym,R,Rsym,x0,xRef,Xlb,Xub,Ulb,Uub,h,tspan,N,T,nx,nu)
%% Multiple shooting, Linear model
%% Multiple shooting is a method to set up a non linear program to be solved by an
%% inferior point solver, here IPOPT. This function takes in an estimated model, 
%% constraints on states and inputs, costfunction matricies, and MPC-parameters, for then
%% to solve and simulate the resulting system with an MPC-controller. 

%% Initialization
    nlpProb = NLPsetup(nx,nu,N,T,A,B,Q,R);
%% Options
    [opts,args] = options(nx,nu,N,Xlb,Xub,Ulb,Uub);
    solver = casadi.nlpsol('solver', 'ipopt', nlpProb,opts);   

%% Simulation loop
%------Output Variables-----
    x = zeros(tspan/h,size(x0,1));      %Matrix of resulting states
    u = zeros(tspan/h,1);               %Matrix of resulting inputs
    xopttmp = zeros(tspan/h*nx,N+1);    %Matrix of all optimal trajetories at each optitime t
    uopttmp = zeros(tspan/h*nu,N);      %Matrix of all optimal inputs at each optitime t
    
    X0 = zeros(2,N+1);                  %Initial search area for X variables
    U0 = zeros(N,nu);                   %Initial search area for U variables
    xx = zeros(nx,N);                   %Vector of current optimal trajectory
    uu = zeros(nu,N);                   %Vector of current optimal input

  
    x(1,:) = x0;
	mpciter = 0;                        %Counter for tracking simulation iterations 
    inputTime = 0;                      %Timer for when new optimalization is due.
%Start of simulation   
    while(norm((x0-xRef),2)> 1e-3 && mpciter<tspan/h)
        mpciter = mpciter + 1;
        if h*(mpciter-1)>=inputTime
            % Set the current values of references and x0 in args.p
            % Set the search space for next opti in args.x0
            args.p = [x0;xRef]; 
            args.x0 = [reshape(X0',nx*(N+1),1); reshape(U0',nu*N,1)];
            
            %Solves the NLP
            sol = solver('x0',args.x0, 'lbx',args.lbx,'ubx',args.ubx,'lbg',...
            args.lbg,'ubg',args.ubg,'p',args.p);
            
            %Extract inputs and path. Disp in proper form
            uu  = reshape(full(sol.x(nx*(N+1)+1:end))',nu,N); 
            xx  = reshape(full(sol.x(1:nx*(N+1)) ),nx,N+1);     
            
            %Store optimal solution in matrix
            xopttmp((mpciter-1)*nx + 1:(mpciter-1)*nx +nx ,1:N+1) = xx;
            uopttmp((mpciter-1)*nu + 1:(mpciter-1)*nu +nu ,1:N) = uu;
            
            %Next optimalization time
            inputTime = inputTime + T/N;
            [LSym,xSym,uSym,chiSym,cost] = LagrangianPhi(T,N,nx,nu,Q,Qsym,R,Rsym,A,B,xx,uu,sol,args,nlpProb,mpciter);            
        else 
            %No optimalization, store as nan in order to ease plotting
            xopttmp((mpciter-1)*nx + 1:(mpciter-1)*nx +nx ,1:N+1) = nan;
            uopttmp((mpciter-1)*nu + 1:(mpciter-1)*nu +nu ,1:N) = nan;            
        end

        %Apply the input and store applied input at time t
        x = RK4(x,uu(:,1),h,mpciter);
        u(mpciter) = uu(:,1);
        
        %declare new inital conditions and search space, process to next iteration
        x0 = x(mpciter+1,:)';
        X0 = xx;
        
        
    end
    

    xopt = zeros(mpciter*nx,N+1);
    uopt = zeros(mpciter*nu,N);
    
    %Reshaping the storage matrix s.t it is on the form x1:;x2:;...
    for i = 1:nx
       xopt((i-1)*mpciter +1:i*mpciter ,:) = reshape(xopttmp(i:nx:mpciter*nx,:),size(xopttmp(i:nx:mpciter*nx,:),1),N+1);
    end
    for i = 1:nu
       uopt((i-1)*mpciter +1:i*mpciter ,:) = reshape(uopttmp(i:nu:mpciter*nu,:),size(uopttmp(i:nu:mpciter*nu,:),1),N);
    end    
    %Extracing only valid points, if the simulation was finished before
    %tspan
    t = (0:h:(mpciter)*h + T)';
    x = x(1:mpciter,:);
    u = u(1:mpciter,:);
    
end

function nlpProb = NLPsetup(nx,nu,N,T,A,B,Q,R)
%% This function declares the primary cost,states,and constraints, and thereby defines
%% the NLPproblem 
%-----------Define the states and inputs---------
    X  = casadi.SX.sym('X',nx,N+1); %State/decision variables
    U  = casadi.SX.sym('U',nu,N);   %Input/decision variables
    P = casadi.SX.sym('P',nx*2);    %Inital and reference values
    stageCost = casadi.SX(N,1);     %Vector of Stagecost/objective function
    obj = 0;                        %Objective function -> sum of stageCost
    g2 = casadi.SX(nx*N+nx,1);      %Vector of equality constraints
    
%----------Setup of NLP--------------    
%Calculation of stagecost, Objective function, and constraints
    g2(1:nx) = X(:,1) - P(1:nx);
    for i = 1:N
       g2(i*nx+1:(i+1)*nx) = X(:,i+1) - (X(:,i) + T/N*(A*X(:,i) + B*U(:,i)));
       
       l 
       
       %stageCost(i) = 0.5*(X(:,i+1))'*Q*(X(:,i+1)) + 0.5*U(:,i)'*R*(U(:,i));
       
       obj = obj + stageCost(i);
       
    end
% Define Optimization variables and the nlpProblem
    OPTVariables = [reshape(X,nx*(N+1),1); reshape(U,nu*N,1)];
    nlpProb = struct('f',obj,'x',OPTVariables, 'g',g2,'p',P);
%--------Arguments & Options------------

end

function  [opts,args]= options(nx,nu,N,Xlb,Xub,Ulb,Uub)
%% This function declares options for the solver (IPopt), and prepares other
%% arguments, such as bounds on states, for the Solver. 
%Options
    opts = struct;
    opts.ipopt.max_iter = 100;
    opts.ipopt.print_level = 0;
    opts.print_time = 0;
    opts.ipopt.acceptable_tol = 1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;
%Arguments
    args = struct;
% lbg&ubg = lower and upper bounds on the equality constraint
    args.lbg(1:nx*(N+1),1) = 0; 
    args.ubg(1:nx*(N+1),1) = 0;

%lbx&ubx = lower and upper bounds on the decision states;
%Checking for any upper or lower bounds, and adding them to args if
%existing.
    for i = 1:nx
        if ~isempty(Xlb) % Lower bounds on X
            args.lbx(i:nx:nx*(N+1),1) = Xlb(i); 
        else
            args.lbx(i:nx:nx*(N+1),1) = -inf; 
        end
        if ~isempty(Xub) % Upper bounds on X
            args.ubx(i:nx:nx*(N+1),1) = Xub(i);
        else
            args.ubx(i:nx:nx*(N+1),1) = inf;             
        end        
    end
    for i  = 1:nu
        if ~isempty(Ulb) % Lower bounds on U
            args.lbx(nx*(N+1)+i:nu:nx*(N+1) + nu*N,1) = Ulb(i); 
        else
            args.lbx(nx*(N+1)+i:nu:nx*(N+1) + nu*N,1) = -inf; 
        end
        if ~isempty(Uub) % Upper bounds on U
            args.ubx(nx*(N+1)+i:nu:nx*(N+1) + nu*N,1) = Uub(i); 
        else
            args.ubx(nx*(N+1)+i:nu:nx*(N+1) + nu*N,1) = inf;             
        end       
    end
    

end

function x = RK4(x,u,dt,t)
%Numerical Simulation method, Runge kutte of 4th order
    k1 = dt*PlantMDS(x(t,:)',u);
    k2 = dt*PlantMDS(x(t,:)' + k1/2,u);
    k3 = dt*PlantMDS(x(t,:)' + k2/2,u);
    k4 = dt*PlantMDS(x(t,:)' + k3,u);
    x(t+1,:) = x(t,:) + 1/6*(k1 + 2*k2 + 2*k3 + k4)';
end
    
function [L,x,u,chi,cost] = LagrangianX(T,N,nx,nu,Q,Qsym,R,Rsym,A,B,xx,uu,sol,args,nlpProb)
%% This function calculates the symbolic lagrangian at the given optimum x*. IT thereafter
%% prints the KKT-conditions, and if no errors, then the Gradient should be all zero

%Number of eqconstraints = N*nx  <=> Number of lagrange multipliers for eq
%Solver uses X0 as a decision variable, and an equality constraint? to 
%Ensure that it = x0. nx new multipliers and nx constraints are therefore
%added
%% Initialization of variables
    x0 = xx(:,1);
    syms x   [N+1 nx] real; x=x';           %State variables
    syms u   [N+1 nu]   real; u=u';         %Input variables
    syms chi [N+1 nx] real;                 %Lagrange multipliers for function eq constraints
    z = [x(:,2:end);u(1:end-1)];            %Decision variables vector(That are in the cost)
    vars = [x;u;chi'];                      %Matrix of differentiable variables
    nums = full([xx;[uu,0];reshape(sol.lam_g,2,N+1)]);% Matrix of numerical values for diff variables
    chi = reshape(chi',(N+1)*nx,1);         %Reshaping the multipliers to correct form
    cost = sym('0');                        %Cost / stage function
    constraints(1:N*nx,1) = sym('0');       %Vector of all equality constraints
%% Calculation of symbolic lagrangian
    for k = 1:N % Sum of cost, from 0 to N-1
        cost = cost + 0.5*(z(1:nx,k)'*Q*z(1:nx,k) + z(nx+1:nx+nu,k)'*R*z(nx+1:nx+nu,k));
        constraints((k-1)*nx+1:k*nx) = x(:,k) +  T/N*(A*x(:,k) + B*u(:,k)) - x(:,k+1);
    end
    constraints = [x0 - x(:,1);constraints];
    L = cost - chi'*constraints;


%% Calculation of symbolic gradients
    J(nx+nu,N+1,1) = sym('0');
        Jrow(1,N+1) = sym('0');
    for row = 1:nx+nu

        for col = 1:N+1
            Jrow(1,col) = diff(L,vars(row,col),1);
        end
        J(row,:) = Jrow;
    end
    fprintf("Solution cost                 : %6.2f \n",full(sol.f))
    fprintf("Solution cost - evaluated cost: %6.2f \n",full(sol.f)-eval(subs(cost,vars,nums)))
    fprintf("Symbolic Lagrangian           : %s \n",L)
    fprintf("Symbolic Gradient at optimum  : \n \n")
        disp(J)
    fprintf("Numerical Gradient at optimum : \n \t")
        disp(eval(subs(J,vars,nums)))

end

function [L,x,u,chi,cost] = LagrangianPhi(T,N,nx,nu,Q,Qsym,R,Rsym,A,B,xx,uu,sol,args,nlpProb,mpciter)
%% This function calculates the symbolic gradient with respect to parametric variables in
%% the cost function, such as Q and R. It is used to view the sensitivities of the solution, 
%% given the tuning parameters Q and R 

%% Initialization of variables
    x0 = xx(:,1);
    phi = [reshape(Qsym,nx*nx,1);reshape(Rsym,nu*nu,1) ]; 
    syms x   [N+1 nx] real; x=x';           %State variables
    syms u   [N+1 nu]   real; u=u';         %Input variables
    syms chi [N+1 nx] real;                 %Lagrange multipliers for function eq constraints
    z = [x(:,2:end);u(1:end-1)];            %Decision variables vector(That are in the cost)
    vars = [x;u;chi'];                      %Matrix of differentiable variables
    nums = full([xx;[uu,0];reshape(sol.lam_g,2,N+1)]);% Matrix of numerical values for diff variables
    chi = reshape(chi',(N+1)*nx,1);         %Reshaping the multipliers to correct form
    cost = sym('0');                        %Cost / stage function
    constraints(1:N*nx,1) = sym('0');       %Vector of all equality constraints
%% Calculation of symbolic lagrangian
    for k = 1:N % Sum of cost, from 0 to N-1
        cost = cost + 0.5*(z(1:nx,k)'*Qsym*z(1:nx,k) + z(nx+1:nx+nu,k)'*Rsym*z(nx+1:nx+nu,k));
        constraints((k-1)*nx+1:k*nx) = x(:,k) +  T/N*(A*x(:,k) + B*u(:,k)) - x(:,k+1);
    end
    constraints = [x0 - x(:,1);constraints];
    L = cost - chi'*constraints;


%% Calculation of symbolic gradients
    J(size(phi,1),1) = sym('0');
    for i = 1:size(phi,1)
        J(i) = diff(L,phi(i),1);
    end
    J;

    
    fprintf("Symbolic Lagrangian           : %s \n",L)
    fprintf("Symbolic GradientPhi          : \n \n")
        disp(J)
    fprintf("Numerical GradientPhi         : \n ")
        disp(eval(subs(J,vars,nums)))
    figure(2)
    hold on; grid on; axis([0 1000 -2 2])
    plot(mpciter,nums(1,1),'k.','Tag','Point')
    
    t = findobj;
    for i = 1:size(t,1)
        if t(i).Tag == "Opt_Traj"
            delete(t(i))
        end
    end
    
    plot(mpciter:T/0.01/N:T/0.01 + mpciter,nums(1,:),'Tag',"Opt_Traj")
    hold off;

end
   
    
    

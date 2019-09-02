%function  Simulation(A,B,P,N,dt,n,tspan,h,x0)
%% Multiple shooting, Linear model
% nx = n states, nu = n inputs, N = Prediciton horizon, dt = predict res
% A,B = System matricies; Q,R = Cost matrices, x0 = System initial cond
% xGuess = initial guess
nx = 4; nu = 1;
N = 30; dt = 0.1; 
A = [0,1,0,0;0,0,4.905,0;0,0,0,1;0,0,14.715,0];
B = [0;0.5;0;0.5];    
Q = diag([50,1,1,1]);
R = 0;
x0 = [1;0;0.7;0];
xRef= [0;0;0;0];
Xlb = []; Xub = [];
Ulb = []; Uub = [];

%[x,u,t,uopt,xopt] =
%% Initialization
%-----------Define the states and inputs---------
    %X = State variables, U = control inputs, xnext = Euler progression
    %Obj = Cost function %g2 = Equality constraints (System dynamics);
    %g1 = Inequality constraints (inputs/state limits) 
    X  = casadi.SX.sym('x',nx,N+1);
    U  = casadi.SX.sym('u',nu,N);
    X0 = casadi.SX(n,1);
    XRef = casadi.SX(n,1);
    P = [X0;XRef];
    xnext = casadi.SX(4,N);
    stageCost = casadi.SX(N,1);
    obj = 0;
    g1 = 0;
    g2 = casadi.SX(n*N+n,1);
    
%----------Setup of NLP--------------    
    %Constraints: 
    g2(1:4) = X(:,1) - X0;
    for i = 1:N
       xnext =X(:,i) + dt*(A*X(:,i) + B*U(:,i));
       stageCost(i) = (X(:,i)-XRef)'*Q*(X(:,i)-XRef) + U(:,i)'*R*(U(:,i));
       obj = obj + stageCost(i);
       g2(i*n+1:(i+1)*n) = X(:,i+1) - xnext;
    end
    %Optimization variables
    OPTVariables = [reshape(X,nx*(N+1),1); reshape(U,nu*N,1)];
    nlpProb = struct('f',obj,'x',OPTVariables, 'g',g2,'p',P);
 
%--------Arguments & Options------------
    opts = struct;
    opts.ipopt.max_iter = 100;
    %opts.ipopt.print_level = 0;
    %opts.print_time = 0;
    opts.ipopt.acceptable_tol = 1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;
    solver = casadi.nlpsol('solver', 'ipopt', nlpProb,opts);   
    % lbg&ubg = lower and upper bounds on the equality constraint
    % lbx&ubx = lower and upper bounds on the decision states;
    args = struct;
    args.lbg(1:nx*(N+1),1) = 0;
    args.ubg(1:nx*(N+1),1) = 0;
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
    
%% Simulation loop
%------Output Variables-----
    t = [0;tspan];
    x = zeros(tspan/h,size(x0,1));
    u = zeros(tspan/h,1);
    uopt = zeros(tspan/h,N);
    xopt = zeros(tspan/h,N+1);
    x(1,:) = x0;
	mpciter = 1; 
    
    X0 = repmat(x0,1,N+1); U0 = zeros(N,nu); % Search initial conditions
    while(norm((x0-xRef),2)> 1e-2 && mpciter<tspan/h)
        args.p = [x0;xRef]; % Set the current values of reference and x0
        args.x0 = [reshape(X0',nx*(N+1),1); reshape(U0',nu*N,1)];
 
        sol = solver('x0',args.x0, 'lbx',args.lbx,'ubx',args.ubx,'lbg',...
        args.lbg,'ubg',args.ubg,'p',args.p);
        
        %Extract inputs and path. Disp in proper form
        u  = reshape(full(sol.x(nx*(N+1)+1:end))',nu,N) 
        xx = reshape(full(sol.x(1:nx*(N+1)) ),nx,N+1)
        
        %Apply the input
        x = RK4(x,u(:,1),h,mpciter);
        
        % Declare new inital conditions
        mpciter = mpciter + 1;
        x0 = x(mpciter,:)';
        X0 = [xx(2:end,:);xx(end,:)];
        norm((x0-xRef),2)
        
        
    end

function x = RK4(x,u,dt,t)
    k1 = dt*Plant(x(t,:)',u);
    k2 = dt*Plant(x(t,:)' + k1/2,u);
    k3 = dt*Plant(x(t,:)' + k2/2,u);
    k4 = dt*Plant(x(t,:)' + k3,u);
    x(t+1,:) = x(t,:) + 1/6*(k1 + 2*k2 + 2*k3 + k4)';
end
    
    
        
   
    
    

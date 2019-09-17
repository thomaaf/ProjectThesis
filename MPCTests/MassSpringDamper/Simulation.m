function [x,u,xopt,uopt,t]= Simulation(A,B,Q,R,x0,xRef,Xlb,Xub,Ulb,Uub,h,tspan,N,T,nx,nu)
%% Multiple shooting, Linear model
%% Initialization
%-----------Define the states and inputs---------
    %X = State variables, U = control inputs, xnext = Euler progression
    %Obj = Cost function %g2 = Equality constraints (System dynamics);
    %g1 = Inequality constraints (inputs/state limits) 
    X  = casadi.SX.sym('X',nx,N+1);
    U  = casadi.SX.sym('U',nu,N);

    P = casadi.SX.sym('P',nx*2);
   % xnext = casadi.SX(nx,N);
    stageCost = casadi.SX(N,1);
    obj = 0;
    g1 = 0;
    g2 = casadi.SX(nx*N+nx,1);
    
%----------Setup of NLP--------------    
    %Constraints: 

    g2(1:nx) = X(:,1) - P(1:nx);
    for i = 1:N

       xnext = X(:,i) + T/N*(A*X(:,i) + B*U(:,i));
       %stageCost(i) = 0.5*(X(:,i)-P(nx+1:end))'*Q*(X(:,i)-P(nx+1:end)) + 0.5*U(:,i)'*R*(U(:,i));
       stageCost(i) = 0.5*(X(:,i+1))'*Q*(X(:,i+1)) + 0.5*U(:,i)'*R*(U(:,i));
       
       obj = obj + stageCost(i);
       i*nx+1:(i+1)*nx
       g2(i*nx+1:(i+1)*nx) = X(:,i+1) - xnext;
    end

    %Optimization variables
    OPTVariables = [reshape(X,nx*(N+1),1); reshape(U,nu*N,1)];
    nlpProb = struct('f',obj,'x',OPTVariables, 'g',g2,'p',P);
%--------Arguments & Options------------
    opts = struct;
    opts.ipopt.max_iter = 100;
    opts.ipopt.print_level = 5;
    opts.print_time = 1;
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
    uopttmp = zeros(tspan/h*nu,N);
    xopttmp = zeros(tspan/h*nx,N+1);
    x(1,:) = x0;
	mpciter = 1; 
    X0 = repmat(x0,1,N+1); U0 = zeros(N,nu); 
    X0 = zeros(2,N+1);
    uu = zeros(nu,N); xx = zeros(nx,N);
    inputTime = 0;
    while(norm((x0-xRef),2)> 1e-3 && mpciter<tspan/h)
        t = h*(mpciter-1);
        if t>=inputTime
            args.p = [x0;xRef]; % Set the current values of reference and x0
            args.x0 = [reshape(X0',nx*(N+1),1); reshape(U0',nu*N,1)];
            sol = solver('x0',args.x0, 'lbx',args.lbx,'ubx',args.ubx,'lbg',...
            args.lbg,'ubg',args.ubg,'p',args.p);
            disp("Optimal Input")
            sol
            %Extract inputs and path. Disp in proper form
            uu  = reshape(full(sol.x(nx*(N+1)+1:end))',nu,N); 
            xx  = reshape(full(sol.x(1:nx*(N+1)) ),nx,N+1);     
            
            xopttmp((mpciter-1)*nx + 1:(mpciter-1)*nx +nx ,1:N+1) = xx;
            uopttmp((mpciter-1)*nu + 1:(mpciter-1)*nu +nu ,1:N) = uu;
            inputTime = inputTime + T/N;
            [LSym,xSym,uSym,chiSym,cost] = Lagrangian(T,N,nx,nu,Q,R,A,B,xx,uu,sol,args);            
        else 
            xopttmp((mpciter-1)*nx + 1:(mpciter-1)*nx +nx ,1:N+1) = nan;
            uopttmp((mpciter-1)*nu + 1:(mpciter-1)*nu +nu ,1:N) = nan;            
        end

        %Apply the input
        x = RK4(x,uu(:,1),h,mpciter);
        u(mpciter) = uu(:,1);
        % Declare new inital conditions
        mpciter = mpciter + 1;
        x0 = x(mpciter,:)';
        
        %X0 = [xx(2:end,:);xx(end,:)];
        X0 = xx;
        

        
    end
    mpciter = mpciter - 1;
    xopt = zeros(mpciter*nx,N+1);
    uopt = zeros(mpciter*nu,N);
    for i = 1:nx
       xopt((i-1)*mpciter +1:i*mpciter ,:) = reshape(xopttmp(i:nx:mpciter*nx,:),size(xopttmp(i:nx:mpciter*nx,:),1),N+1);
    end
    for i = 1:nu
       uopt((i-1)*mpciter +1:i*mpciter ,:) = reshape(uopttmp(i:nu:mpciter*nu,:),size(uopttmp(i:nu:mpciter*nu,:),1),N);
    end    
    t = (0:h:(mpciter)*h + T)';
    x = x(1:mpciter+1,:);
    u = u(1:mpciter+1,:);
    
end

function x = RK4(x,u,dt,t)
    k1 = dt*PlantMDS(x(t,:)',u);
    k2 = dt*PlantMDS(x(t,:)' + k1/2,u);
    k3 = dt*PlantMDS(x(t,:)' + k2/2,u);
    k4 = dt*PlantMDS(x(t,:)' + k3,u);
    x(t+1,:) = x(t,:) + 1/6*(k1 + 2*k2 + 2*k3 + k4)';
end
    
    
function [L,x,u,chi,cost] = Lagrangian(T,N,nx,nu,Q,R,A,B,xx,uu,sol,args)
    syms x   [1 (N+1)*nx] real   %State variables
    syms u   [1 (N+1)*nu] real  %Input variables
    syms chi [1 (N+1)*nx] real   %Lagrange multipliers for function eq constraints
    x   = reshape(x,nx,N+1);
    u   = reshape(u,nu,N+1); %Extra padding u added for matrixinclusion
    %chi = reshape(chi,nx,N+1);
	vars = [x;reshape(chi,nx,N+1);u];
    nums = [xx;full(reshape(sol.lam_g,nx,N+1));uu,0]; %Last additional element of u set equal to 0
    cost = sym('0');
    constraints = x(:,1) - args.p(1:2);
    %constraints = [x(:,1) = ];
    for k = 1:N
       cost = cost + 0.5*x(:,k+1)'*Q*x(:,k+1) + 0.5*u(:,k)'*R*u(:,k); 
       constraints(k*nx+1:(k+1)*nx) = (eye(2) + T/N*A)*x(:,k) + T/N*B*u(:,k) - x(:,k+1);
       %X = X + chi(:,k+1)'*(A*x(:,k) + B*u(:,k) - x(:,k+1)); 
       
    end
    L  = chi*constraints + cost;
    fprintf("Sol.Cost = %6.2f \nCus.Cost = %6.2f\n",full(sol.f),eval(subs(cost,x,xx)))
    fprintf("Lagrangian: %s \n",L)
    fprintf("Evaluation of Lagrangian:\n \t %6.2f \n",eval(subs(L,vars,nums)))
    fprintf("Evaluation of gradient_X: \n")
    J = [];
    J1 = [];
    J2 = [];
    for i = 1:(N+1)
        J1 = [J1, diff(L,x(1,i),1)];
        J2 = [J2, diff(L,x(2,i),1)];
        
    end
    J1 = (eval(subs(J1,vars,nums)));
    J2 = (eval(subs(J2,vars,nums)));
    J = [J1;J2]
    
    
    
    
    


end
   
    
    

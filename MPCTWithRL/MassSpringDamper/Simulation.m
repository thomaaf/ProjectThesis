function out = Simulation(Model,MPCParam,RLParam,InitParam)
%% Multiple shooting, Linear model
%% Multiple shooting is a method to set up a non linear program to be solved by an
%% inferior point solver, here IPOPT. This function takes in an estimated model, 
%% constraints on states and inputs, costfunction matricies, and MPC-parameters, for then
%% to solve and simulate the resulting system with an MPC-controller. 

%% Initialization
    nx = Model.nx; nu = Model.nu;
    N = MPCParam.N; T = MPCParam.T;
    tspan = InitParam.tspan;
    h  = InitParam.h;
    x0 = InitParam.x0; xRef = InitParam.xRef;

    nlpProb = NLPsetup(Model,MPCParam,RLParam,InitParam);
    
    
%% Options
    [opts,args] = casadiOptions(Model,MPCParam,RLParam,InitParam);
    solver = casadi.nlpsol('solver', 'ipopt', nlpProb,opts);   

%% Simulation loop
% Reshape counts row-wise.so for col; for row
%------Output Variables-----
    x       = zeros(tspan/h,nx);                    %Matrix of resulting states
    u       = zeros(tspan/h,1);                     %Matrix of resulting inputs
    V       = zeros(tspan/h,1); 
    nabla   = zeros(tspan/h,size(RLParam.theta,1));      
    TD      = zeros(tspan/h,1);                     %Vector of the Temporal differences
    theta   = zeros(tspan/h,MPCParam.nSym-nx^2); %Matrix of theta variables 
    xopt    = zeros(tspan/h*nx,N+1);                %Matrix of all optimal trajetories at each optitime t
    uopt    = zeros(tspan/h*nu,N);                  %Matrix of all optimal inputs at each optitime t      
    numsopt = zeros(tspan/h*nx,size(MPCParam.vars,1));
    reward       = zeros(tspan/h,1);
    target   = zeros(tspan/h,1);


%----Misc initialization------
    args.X0 = zeros(2,N+1);                              %Initial search area for X variables
    args.U0 = zeros(N,nu);                               %Initial search area for U variables
    [timelabel,statelabel,~,Parameternumlabel]= printStateInit(theta(1,:),0,x0,RLParam); 
    %x(1,:) = x0;
    simcount =1;                            %Counter for tracking simulation iterations 
    inputTime = T/N;                        %Timer for when new optimalization is due.
    optCount = 0;                           %Counter for tracking optimizations
    
    
%-----Calculations of initial values-----------    
% theta = [Q,A,R,B,E,F]
    theta_s = [reshape(MPCParam.Q',nx*nx,1);...  %Initial values for theta
                  reshape(Model.A',nx*nx,1);...
                  reshape(MPCParam.R',nu*nu,1);...
                  reshape(Model.B',nx*nu,1);...
                  reshape(Model.E',nx,1);...
                  reshape(MPCParam.f',nx+nu,1);...
                  MPCParam.V0];                    
    
    s = x0;
    [Plqr, nums,Vs,a] = solveOpt(solver,args,MPCParam,Model,theta_s,s,xRef);    
    [nabla_s,nums] = numericalGradiant(MPCParam,"F",nums,Plqr,theta_s);
    Rs = s'*RLParam.Q*s + a'*RLParam.R*a; 
    TDs = 0;   target_s = 0; 
    
    numsn = nums;
    an = a;
    Vsn = Vs;
    TDn = TDs;
    theta_n = theta_s ;
    nabla_n = nabla_s;
    Rn = Rs;
    target_n = target_s;
%Start of simulation   
%(norm((x0-xRef),2) > 1e-3 &&
    while simcount<tspan/h
           
        sn = RK4(s,a,h); %Take action a, obsereve next state Sn
        
        if (h*(simcount)>=inputTime + 0.0001) || (h*(simcount)>=inputTime - 0.0001)
            %Calculate V(sn), and also an = pi(sn)
            [Plqr, numsn,Vsn,an] = solveOpt(solver,args,MPCParam,Model,theta_s,sn,xRef);
            
            
            [KKT,~] = numericalGradiant(MPCParam,"KKTF",numsn,Plqr,theta_s);
            
            [nabla_n,numsn] = numericalGradiant(MPCParam,"F",numsn,Plqr,theta_s);
            
            
            [TDn,theta_n,Rn,target_n] = RLUpdate(theta_s,Vs,Vsn,sn,an,nabla_s,RLParam);
            
            printState(theta_s,(simcount-T/N/h)*h,s,timelabel,statelabel,Parameternumlabel,TDs,a)
            if FeasabilityCheck(KKT,opts.ipopt.acceptable_tol,theta_s)
               break;
            end
            inputTime = inputTime + T/N; 
            args.X0 = numsn(1:nx*(N+1));   
            args.U0 = numsn(nx*(N+1)+1:nx*(N+1) + nu*(N));          
        end      
        numsopt(simcount,:) = nums';
        x(simcount,:)       = s';
        u(simcount,:)       = a';
        V(simcount,:)       = Vs;
        nabla(simcount,:)   = nabla_s';
        TD(simcount,:)      = TDs;
        theta(simcount,:)   = theta_s';
        xopt((simcount-1)*nx + 1:(simcount-1)*nx +nx ,1:N+1) = reshape(nums(1:nx*(N+1)),N+1,nx)';
        uopt((simcount-1)*nu + 1:(simcount-1)*nu +nu ,1:N)   = reshape(nums(nx*(N+1)+1:nx*(N+1) + nu*(N)),N,nu)';
        reward(simcount,:)       = Rs;
        target(simcount,:)   = target_s;
        
        nums = numsn;
        s = sn;
        a = an;
        Vs = Vsn;
        TDs = TDn;
        theta_s = theta_n;
        nabla_s = nabla_n;
        Rs = Rn;
        target_s = target_n;
        simcount = simcount + 1;
    end
    
    simcount = simcount - 1;
    xOptNew = zeros(simcount*nx,N+1);
    uOptNew = zeros(simcount*nu,N);

    %Reshaping the storage matrix s.t it is on the form x1:;x2:;...
    for i = 1:nx
       xOptNew((i-1)*simcount+1:((i-1)*simcount+simcount),:) = xopt(i:nx:simcount*nx,:);
    end
    for i = 1:nu
       uOptNew((i-1)*simcount+1:((i-1)*simcount+simcount),:) = uopt(i:nu:simcount*nu,:);
    end    

    
    %Extracing only valid points, if the simulation was finished before
    %tspan
    t2 = (0:h:(simcount)*h + T)';
    out = struct('x',x(1:simcount,:),'u',u(1:simcount,:),'t',t2,'xopt',xOptNew,'uopt',uOptNew...
        ,'theta',theta(1:simcount,:),'V',V(1:simcount,:),'nabla',nabla(1:simcount,:)...
        ,'TD',TD(1:simcount),'simcount',simcount,'numsopt',numsopt(1:simcount,:)...
        ,'reward',reward(1:simcount,:),'target',target(1:simcount,:));
    

end


function flag = FeasabilityCheck(KKT,acceptable_tol,theta)
    A = [theta(5:6)';theta(7:8)'];
    Q = [theta(1:2)';theta(3:4)'];    
    flag = 0;
    if max(KKT)> acceptable_tol
        fprintf("Aborted due to an error in estimated KKT conditions\n");
        fprintf( ['The Prime conditions should be zero but is : ' repmat('% -6.4f  |', 1, size(KKT,1)) '\n'], KKT')                
        flag = 1;
    elseif det(Q)<0.001 
        fprintf("Aborted due to loss of convexity or stability\n");          
        flag = 1; 
    end 
end

function [Plqr, nums,Vs,a] = solveOpt(solver,args,MPCParam,Model,theta,s,xRef)
% P = [Plqr,Q,A,R,B,E,F,X0,Xref]
    nx = Model.nx; nu = Model.nu; N = MPCParam.N; 
    Q    = reshape(theta(0*nx*nx + 0*nu*nu + 0*nx + 1 : 1*nx*nx + 0*nu*nu),nx,nx)';
    A    = reshape(theta(1*nx*nx + 0*nu*nu + 0*nx + 1 : 2*nx*nx + 0*nu*nu),nx,nx)';
    R    = reshape(theta(2*nx*nx + 0*nu*nu + 0*nx + 1 : 2*nx*nx + 1*nu*nu),nu,nu)';
    B    = reshape(theta(2*nx*nx + 1*nu*nu + 0*nx + 1 : 2*nx*nx + 1*nu*nu + 1*nx*nu),nu,nx)';
    [~,Plqr,~] = lqr(A,B,Q,R,[]);
    %Plqr = [0 0; 0 0];
    Plqr = reshape(Plqr',nx*nx,1);

    args.p = [Plqr;theta;s;xRef];   
    args.x0 = [reshape(args.X0',nx*(N+1),1); reshape(args.U0',nu*N,1)];       % Set the search space for next opti in args.x0
%---------------solve next optimization-----------------------------------            
    sol = solver('x0',args.x0, 'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
    Vs = full(sol.f);
    uu  = reshape(full(sol.x(nx*(N+1)+1:end))',nu,N);	% Extract inputs
    xx  = reshape(full(sol.x(1:nx*(N+1)) ),nx,N+1);     % Extract states
    nums = [reshape(xx',nx*(N+1),1);reshape(uu',nu*(N),1);full(sol.lam_g)*-1];  % Extract all numerical data
    a = uu(1);
end

function nlpProb = NLPsetup(Model,MPCParam,RLParam,InitParam)
% Redfining used parameters in order to reduce code size and increase
% readability
%Model
    nx = Model.nx; nu = Model.nu;
%MPC
    N = MPCParam.N; T = MPCParam.T;
%RL
    gamma = RLParam.gamma;
    

%% This function declares the primary cost,states,and constraints, and thereby defines
%% the NLPproblem. This must be modified with each Learned parameter to be included in P,st
%% P is the size of nx*2 + size of theta + any other changes, such as LQR size

%-----------Define the states and inputs---------
% P: nx*2 for init and ref. nx*nx*2 for Q and LQR, nx*nx for A
    X  = casadi.SX.sym('X',nx,N+1); %State/decision variables
    U  = casadi.SX.sym('U',nu,N);   %Input/decision variables
    P = casadi.SX.sym('P',MPCParam.nSym + nx*2);    %Inital and reference values
    DCStageCost = casadi.SX(N,1);   %Vector of Discounted Stagecost 
    LStageCost = casadi.SX(N,1);   %Vector of Linear, Parametrized Stagecost 
    obj = 0;                        %Objective function -> sum of stageCost
    g2 = casadi.SX(nx*N+nx,1);      %Vector of equality constraints
%----------Defining the RL-parameters--------------   
    Plqr = reshape(P(0*nx*nx + 0*nu*nu + 0*nx + 1 : 1*nx*nx + 0*nu*nu),nx,nx)';
    %Plqr = [0 0; 0 0];
    Q    = reshape(P(1*nx*nx + 0*nu*nu + 0*nx + 1 : 2*nx*nx + 0*nu*nu),nx,nx)';
    A    = reshape(P(2*nx*nx + 0*nu*nu + 0*nx + 1 : 3*nx*nx + 0*nu*nu),nx,nx)';
    R    = reshape(P(3*nx*nx + 0*nu*nu + 0*nx + 1 : 3*nx*nx + 1*nu*nu),nu,nu)';
    B    = reshape(P(3*nx*nx + 1*nu*nu + 0*nx + 1 : 3*nx*nx + 1*nu*nu + 1*nx*nu),nu,nx)';
    E    = reshape(P(3*nx*nx + 1*nu*nu + 1*nx*nu + 1 : 3*nx*nx + 1*nu*nu + 1*nx*nu + nx),1,nx)';
    F    = reshape(P(3*nx*nx + 1*nu*nu + 1*nx*nu + nx + + 0*nu + 1 : 3*nx*nx + 1*nu*nu + 1*nx*nu + 2*nx + nu),1,nx + nu)';
    V0   = reshape(P(3*nx*nx + 1*nu*nu + 1*nx*nu + 2*nx + 1*nu + 1 : 3*nx*nx + 1*nu*nu + 1*nx*nu + 2*nx + nu + 1),1,1)';
    X0   = reshape(P(3*nx*nx + 1*nu*nu + 1*nx*nu + 2*nx + 1*nu + 2 : 3*nx*nx + 1*nu*nu + 1*nx*nu + 3*nx + nu + 1),1,nx)';
    XRef   = reshape(P(3*nx*nx + 1*nu*nu + 1*nx*nu + 3*nx + 1*nu + 2 : 3*nx*nx + 1*nu*nu + 1*nx*nu + 4*nx + nu + 1),1,nx)';
%----------Discretize matricies-----------
Ak = eye(nx) + T/N*A; Bk = T/N*B; Ek = T/N*E;
%----------Setup of NLP--------------    
%Calculation of stagecost, Objective function, and constraints
    % Sum from k = 0:N-1 <-> k = 1:N
    g2(1:nx) = X(:,1) - X0;
    for k = 1:N
        g2(k*nx+1:(k+1)*nx) = X(:,k+1) - (Ak*X(:,k) + Bk*U(:,k) + Ek);
        DCStageCost(k) = 0.5*gamma^(k-1)*( X(:,k)'*Q*X(:,k) + U(:,k)'*R*U(:,k) );
        LStageCost (k) = F'*[X(:,k);U(:,k)];
        obj = obj + DCStageCost(k) + LStageCost(k);
        
        fprintf("\n%i DC:  ",k);
        disp(DCStageCost(k))
        fprintf("%i  L:  ",k);
        disp(LStageCost(k))    
        fprintf("%i  g2:  ",k);
        disp(g2(k))            
        
    end
    fprintf("\n%i Final:  ",k+1);
    disp(V0 + 0.5*gamma^N*X(:,k+1)'*Plqr*X(:,k+1))            
    obj = obj + V0 + 0.5*gamma^N*X(:,k+1)'*Plqr*X(:,k+1);


% Define Optimization variables and the nlpProblem
    OPTVariables = [reshape(X,nx*(N+1),1); reshape(U,nu*N,1)];
    nlpProb = struct('f',obj,'x',OPTVariables, 'g',g2,'p',P);
%--------Arguments & Options------------

end
   



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
    theta   = zeros(tspan/h,size(RLParam.theta,1)); %Matrix of theta variables 
    xopt    = zeros(tspan/h*nx,N+1);                %Matrix of all optimal trajetories at each optitime t
    uopt    = zeros(tspan/h*nu,N);                  %Matrix of all optimal inputs at each optitime t      

    


%----Misc initialization------
    args.X0 = zeros(2,N+1);                              %Initial search area for X variables
    args.U0 = zeros(N,nu);                               %Initial search area for U variables
    [timelabel,statelabel,~,Parameternumlabel]= printStateInit(theta(1,:),0,x0,RLParam); 
    %x(1,:) = x0;
    simcount =1;                            %Counter for tracking simulation iterations 
    inputTime = T/N;                        %Timer for when new optimalization is due.
    optCount = 0;                           %Counter for tracking optimizations
    
    
%-----Calculations of initial values-----------    
    theta_s = [reshape(MPCParam.Q',nx*nx,1);...  %Initial values for theta
                  reshape(Model.A',nx*nx,1);...
                  reshape(Model.E',nx,1)];                    
    
    s = x0;
    [Plqr, nums,Vs,a] = solveOpt(solver,args,MPCParam,Model,theta_s,s,xRef);    
    nabla_s = numericalGradiant(MPCParam,"F",nums,Plqr,theta_s);
    TDs = 0; TDn = 0;
    Vsn = Vs;
    an = a; 
    theta_n = theta_s;
    nabla_n = nabla_s;
%Start of simulation   
%(norm((x0-xRef),2) > 1e-3 &&
    while simcount<tspan/h
           
        sn = RK4(s,a,h);          
        
        if (h*(simcount)>=inputTime + 0.0001) || (h*(simcount)>=inputTime - 0.0001)
            
            [Plqr, nums,Vsn,an] = solveOpt(solver,args,MPCParam,Model,theta_s,sn,xRef);
            
            KKT = numericalGradiant(MPCParam,"KKTF",nums,Plqr,theta_s);
            
            nabla_n = numericalGradiant(MPCParam,"F",nums,Plqr,theta_s);
            
            [TDn,theta_n] = RLUpdate(theta_s,Vs,Vsn,s,a,nabla_s,RLParam);
            
            printState(theta_s,(simcount-T/N/h)*h,s,timelabel,statelabel,Parameternumlabel,TDs,a)
            if FeasabilityCheck(KKT,opts.ipopt.acceptable_tol,theta_s)
                break;
            end
            inputTime = inputTime + T/N; 
            args.X0 = nums(1:nx,1:N+1);   
            args.U0 = nums(nx+1:nx+nu,1:N);          
        end      
        
        x(simcount,:)       = s';
        u(simcount,:)       = a';
        V(simcount,:)       = Vs;
        nabla(simcount,:)   = nabla_s';
        TD(simcount,:)      = TDs;
        theta(simcount,:)   = theta_s';
        xopt((simcount-1)*nx + 1:(simcount-1)*nx +nx ,1:N+1) = nums(1:nx,1:N+1);
        uopt((simcount-1)*nu + 1:(simcount-1)*nu +nu ,1:N) = nums(nx+1:nx+nu,1:N);     
        
        s = sn;
        a = an;
        Vs = Vsn;
        TDs = TDn;
        theta_s = theta_n;
        nabla_s = nabla_n;
        
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
        ,'TD',TD(1:simcount),'simcount',simcount);
    

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
    nx = Model.nx; nu = Model.nu; N = MPCParam.N; 
    A = [theta(5:6)';theta(7:8)'];
    Q = [theta(1:2)';theta(3:4)'];
    [~,Plqr,~] = lqr(A,Model.B,Q,MPCParam.R,[]);

    args.p = [s;xRef;...                    %update parameters in the NLP
              theta(1:4);...
              reshape(Plqr',4,1);...
              theta(5:8);...
              theta(9:10)];      
    args.x0 = [reshape(args.X0',nx*(N+1),1); reshape(args.U0',nu*N,1)];       % Set the search space for next opti in args.x0
%---------------solve next optimization-----------------------------------            
    sol = solver('x0',args.x0, 'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
    Vs = full(sol.f);
    uu  = reshape(full(sol.x(nx*(N+1)+1:end))',nu,N);	% Extract inputs
    xx  = reshape(full(sol.x(1:nx*(N+1)) ),nx,N+1);     % Extract states
    nums = full([xx;[uu,0];reshape(sol.lam_g,2,N+1)]);  % Extract all numerical data
    a = nums(nx+1:nx+nu,1);
end

function nlpProb = NLPsetup(Model,MPCParam,RLParam,InitParam)
% Redfining used parameters in order to reduce code size and increase
% readability
%Model
    nx = Model.nx; nu = Model.nu;
    A  = Model.A ; B  = Model.B; 
%MPC
    N = MPCParam.N; T = MPCParam.T;
    Q = MPCParam.Q; R = MPCParam.R; 
    f = MPCParam.f;
%RL
    gamma = RLParam.gamma;
    

%% This function declares the primary cost,states,and constraints, and thereby defines
%% the NLPproblem. This must be modified with each Learned parameter to be included in P,st
%% P is the size of nx*2 + size of theta + any other changes, such as LQR size

%-----------Define the states and inputs---------
% P: nx*2 for init and ref. nx*nx*2 for Q and LQR, nx*nx for A
    X  = casadi.SX.sym('X',nx,N+1); %State/decision variables
    U  = casadi.SX.sym('U',nu,N);   %Input/decision variables
    P = casadi.SX.sym('P',nx*2 + nx*nx*2 + nx*nx + nx);    %Inital and reference values
    DCStageCost = casadi.SX(N,1);   %Vector of Discounted Stagecost 
    LPStageCost = casadi.SX(N,1);   %Vector of Linear, Parametrized Stagecost 
    obj = 0;                        %Objective function -> sum of stageCost
    g2 = casadi.SX(nx*N+nx,1);      %Vector of equality constraints
%----------Defining the RL-parameters--------------   
    Q = [P(nx*2+1) P(nx*2+2); P(nx*2+3) P(nx*2+4)];
    Plqr = [P(nx*2+5) P(nx*2+6); P(nx*2+7) P(nx*2+8)];
    A = [P(nx*2+9) P(nx*2+10); P(nx*2+11) P(nx*2+12)];
    E = [P(nx*2+13);P(nx*2+14)];
%----------Setup of NLP--------------    
%Calculation of stagecost, Objective function, and constraints
    % Sum from k = 0:N-1 <-> k = 1:N
    g2(1:nx) = X(:,1) - P(1:nx);
    for k = 1:N
        g2(k*nx+1:(k+1)*nx) = X(:,k+1) - ((eye(nx) + T/N*A)*X(:,k) + T/N*(B*U(:,k) + E));        
        DCStageCost(k) =0.5 * gamma^(k-1) * (X(:,k)'*Q*X(:,k) + U(:,k)'*R*U(:,k));
        LPStageCost(k) = f'*[X(:,k);U(:,k)];
        obj = obj + DCStageCost(k) + LPStageCost(k);
        
    end
    %g2 = [g2;U(:,1) - P(end)];
    obj = obj + 0.5*gamma^(k+1)*X(:,k+1)'*Plqr*X(:,k+1);


% Define Optimization variables and the nlpProblem
    OPTVariables = [reshape(X,nx*(N+1),1); reshape(U,nu*N,1)];
    nlpProb = struct('f',obj,'x',OPTVariables, 'g',g2,'p',P);
%--------Arguments & Options------------

end
   



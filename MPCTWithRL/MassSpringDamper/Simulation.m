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
%------Output Variables-----
    x       = zeros(tspan/h,nx);            %Matrix of resulting states
    u       = zeros(tspan/h,1);             %Matrix of resulting inputs
    xopt    = zeros(tspan/h*nx,N+1);        %Matrix of all optimal trajetories at each optitime t
    uopt    = zeros(tspan/h*nu,N);          %Matrix of all optimal inputs at each optitime t
    TD      = zeros(tspan/h,1);             %Vector of the Temporal differences
    theta   = zeros(tspan/h,size(RLParam.theta,1));
%-----Casadi Variables    
    X0 = zeros(2,N+1);                      %Initial search area for X variables
    U0 = zeros(N,nu);                       %Initial search area for U variables

%----Misc initialization
    Vs = 0;  nabla  = zeros(size(RLParam.theta));
    Vsn = 0; nablan = zeros(size(RLParam.theta));
    x(1,:) = x0;
    theta(1,:) = reshape(MPCParam.Q',4,1);
	simCount = 0;                           %Counter for tracking simulation iterations 
    inputTime = 0;                          %Timer for when new optimalization is due.
    optCount = 0;
    Q = reshape(theta(1,:),2,2);
%Start of simulation   
    while(norm((x0-xRef),2)> 1e-3 && simCount<tspan/h)
        simCount = simCount + 1;
        
        x0 = x(simCount,:)';
        u0 = u(simCount,:);
        (simCount-1)*h
        if (h*(simCount-1)>=inputTime + 0.0001) || (h*(simCount-1)>=inputTime - 0.0001)
            optCount = optCount + 1;                                        % Count how many times we have optimized
%---------------Setup next optimization------------------------------------
            
            [K,Plqr,e] = lqr(Model.A,Model.B,Q,MPCParam.R,[]);
            
            Vs = Vsn;
            nabla = nablan;
  
            args.p = [x0;xRef;reshape(Q',4,1);reshape(Plqr',4,1)];       % Set the current values of references and x0 in args.p
            args.x0 = [reshape(X0',nx*(N+1),1); reshape(U0',nu*N,1)];       % Set the search space for next opti in args.x0
            %MPCParam.Q = [theta(1) 0; 0 theta(2)];
            %MPCParam.R = theta(3);
            %nlpProb = NLPsetup(Model,MPCParam,RLParam,InitParam);
            
 %---------------solve next optimization-----------------------------------           
            %solver = casadi.nlpsol('solver', 'ipopt', nlpProb,opts);   
            sol = solver('x0',args.x0, 'lbx',args.lbx,'ubx',args.ubx,'lbg',... %Solves the NLP
            args.lbg,'ubg',args.ubg,'p',args.p);
            Vsn = sol.f;

 %---------------Formatting results----------------------------------------
            uu  = reshape(full(sol.x(nx*(N+1)+1:end))',nu,N); 
            xx  = reshape(full(sol.x(1:nx*(N+1)) ),nx,N+1);     
            nums = full([xx;[uu,0];reshape(sol.lam_g,2,N+1)]);
            nablan = numericalGradiant(MPCParam,Model,MPCParam.Jtheta,nums);
            % theta(optCount,:) is the current theta stored
            % Vs is the value of the current state we are in
            % Vsn is the value of the state we arrive at after applying uu(:,1), where
            % the stochasticity is dealt with from the initial conditions of NLP, 
            % x0 is the current state we are in, uu(:,1) are the to be
            % applied input, and nabla is the gradient of the Lagrange to
            % the current x0 position
            if optCount>1
                [TD(optCount),theta(optCount,:)] = RLUpdate(theta(optCount-1,:),...
                    Vs,Vsn,x(optCount-1,:)',u(optCount-1,:)',nabla);
                Q = reshape(theta(optCount,:),2,2);
            end
            
            
            
%           KKT = numericalGradiant(MPCParam,Model,MPCParam.KKT,nums);
%           if KKT > 1e-6
%           	fprintf("Aborted due to an error in estimated KKT conditions\n");
%           	fprintf("The Prime conditions should be zero but is : %f",KKT);
%           end            
            xopt((optCount-1)*nx + 1:(optCount-1)*nx +nx ,1:N+1) = xx;
            uopt((optCount-1)*nu + 1:(optCount-1)*nu +nu ,1:N) = uu;
            
 %---------------Apply the input, and move to next state-------------------
            x = RK4(x,uu(:,1),h,simCount);
            u(simCount) = uu(:,1);            
            %Next optimalization time

 %---------------Plan for next optimization--------------------------------            
            inputTime = inputTime + T/N; 
            X0 = xx;   
        else 
            %No optimalization, store as nan in order to ease plotting
            %Apply the input and store applied input at time t
            x = RK4(x,uu(:,1),h,simCount);
            u(simCount) = uu(:,1);
            X0 = xx;   
     
        end              
    end
    

    xOptNew = zeros(optCount*nx,N+1);
    uOptNew = zeros(optCount*nu,N);
    
    %Reshaping the storage matrix s.t it is on the form x1:;x2:;...
    for i = 1:nx
       xOptNew((i-1)*optCount+1:((i-1)*optCount+optCount),:) = xopt(i:nx:optCount*nx,:);
    end
    for i = 1:nu
       uOptNew((i-1)*optCount+1:((i-1)*optCount+optCount),:) = uopt(i:nu:optCount*nu,:);
    end    

    
    %Extracing only valid points, if the simulation was finished before
    %tspan
    t = (0:h:(simCount)*h + T)';
    x = x(1:simCount,:);
    u = u(1:simCount,:);
    out = struct('x',x,'u',u,'t',t,'xopt',xOptNew,'uopt',uOptNew,'theta',theta(1:optCount,:));
    out.TD = TD(1:optCount); out.optCount = optCount
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
%% the NLPproblem 
%-----------Define the states and inputs---------
    X  = casadi.SX.sym('X',nx,N+1); %State/decision variables
    U  = casadi.SX.sym('U',nu,N);   %Input/decision variables
    P = casadi.SX.sym('P',nx*2 + size(RLParam.theta,1)*2);    %Inital and reference values
    DCStageCost = casadi.SX(N,1);   %Vector of Discounted Stagecost 
    LPStageCost = casadi.SX(N,1);   %Vector of Linear, Parametrized Stagecost 
    obj = 0;                        %Objective function -> sum of stageCost
    g2 = casadi.SX(nx*N+nx,1);      %Vector of equality constraints
%----------Defining the RL-parameters--------------   
    Q = [P(nx*2+1) P(nx*2+2); P(nx*2+3) P(nx*2+4)];
    Plqr = [P(nx*2+5) P(nx*2+6); P(nx*2+7) P(nx*2+8)];
%----------Setup of NLP--------------    
%Calculation of stagecost, Objective function, and constraints
    % Sum from k = 0:N-1 <-> k = 1:N
    g2(1:nx) = X(:,1) - P(1:nx);
    for k = 1:N
        g2(k*nx+1:(k+1)*nx) = X(:,k+1) - ((eye(nx) + T/N*A)*X(:,k) + T/N*B*U(:,k) );        
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
 
% function [TD, theta] = RLUpdate(Model,MPCParam,RLParam,InitParam,Vs,Vsn,x,u,xn,un,nabla,theta)
%     Q = MPCParam.Q; R = MPCParam.R;    
%     N = MPCParam.N; 
%     nx = Model.nx; nu = Model.nu;
%     alfa = 0.01;
%     Ltheta = 0;
%     nabla = [0;0;0]
%     for k = 1:N
%         [x(1:nx,k).*x(1:nx,k);0] + [0;0;u(1:nu,k)*u(1:nu,k)]
%     	Ltheta = Ltheta + x(1:nx,k)'*[1 0; 0 1]*x(1:nx,k) + u(1:nu,k)'*1*u(1:nu,k);
%         nabla = nabla + 0.5*.9^k*([x(1:nx,k).*x(1:nx,k);0] + [0;0;u(1:nu,k)*u(1:nu,k)]);
%     end
%     TD = Ltheta + 0.9*Vsn - Vs;
%     fprintf("Temporal difference: %2.2f + 0.9*%2.2f - %f2.2\n",Ltheta,Vsn,Vs)
%     fprintf("Temporal difference: %2.2f\n",TD)
%     theta = theta + alfa*TD*nabla;
%     
% end
% 
%     
    

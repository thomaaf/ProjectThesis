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
    xopt    = zeros(tspan/h*nx,N+1);                %Matrix of all optimal trajetories at each optitime t
    uopt    = zeros(tspan/h*nu,N);                  %Matrix of all optimal inputs at each optitime t
    TD      = zeros(tspan/h,1);                     %Vector of the Temporal differences
    theta   = zeros(tspan/h,size(RLParam.theta,1)); %Matrix of theta variables
%-----Casadi Variables    
    X0 = zeros(2,N+1);                              %Initial search area for X variables
    U0 = zeros(N,nu);                               %Initial search area for U variables

%----RL initializations-------
    Vs = 0;  nabla  = zeros(size(RLParam.theta));   %Initilizing current state values and derivatives
    Vsn = 0; nablan = zeros(size(RLParam.theta));   %Initilizing next state values and derivatives
    
    theta(1,:) = [reshape(MPCParam.Q',nx*nx,1);...  %Initial values for theta
                  reshape(Model.A',nx*nx,1);...
                  reshape(Model.E',nx,1)];
                    
    Q = reshape(theta(1,1:nx*nx),2,2)';                    
    A = reshape(theta(1,nx*nx+1:2*nx*nx),2,2)';                  
    E = reshape(theta(1,2*nx*nx+1:2*nx*nx+nx),2,1); 
%----Misc initialization------
    [timelabel,statelabel,~,Parameternumlabel]= printStateInit(theta(1,:),0,x0,RLParam); 
    x(1,:) = x0;
    simCount = 0;                           %Counter for tracking simulation iterations 
    inputTime = 0;                          %Timer for when new optimalization is due.
    optCount = 0;                           %Counter for tracking optimizations
    
%Start of simulation   
%(norm((x0-xRef),2) > 1e-3 &&
    while simCount<tspan/h
        simCount = simCount + 1;
        x0 = x(simCount,:)';
        if (h*(simCount-1)>=inputTime + 0.0001) || (h*(simCount-1)>=inputTime - 0.0001)
            optCount = optCount + 1;                                       
%---------------Setup next optimization------------------------------------
            [~,Plqr,~] = lqr(A,Model.B,Q,MPCParam.R,[]);
            Vs = Vsn;
            nabla = nablan;
            args.p = [x0;xRef;...                    %update parameters in the NLP
                      reshape(Q',4,1);...
                      reshape(Plqr',4,1);...
                      reshape(A',4,1);...
                      reshape(E',2,1)];      
            args.x0 = [reshape(X0',nx*(N+1),1); reshape(U0',nu*N,1)];       % Set the search space for next opti in args.x0
 %---------------solve next optimization-----------------------------------           
            %solver = casadi.nlpsol('solver', 'ipopt', nlpProb,opts);   
            sol = solver('x0',args.x0, 'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
            Vsn = sol.f;
            uu  = reshape(full(sol.x(nx*(N+1)+1:end))',nu,N);	% Extract inputs
            xx  = reshape(full(sol.x(1:nx*(N+1)) ),nx,N+1);     % Extract states
            nums = full([xx;[uu,0];reshape(sol.lam_g,2,N+1)]);  % Extract all numerical data

%----------------RL-update----------------------------
            if optCount>1
                KKT = numericalGradiant(MPCParam,"KKTF",nums,Plqr,theta(optCount-1,:)');    % Check that KKT not violated
                nablan = numericalGradiant(MPCParam,"F",nums,Plqr,theta(optCount-1,:)');    % Calculate derivative
                [TD(optCount),theta(optCount,:)] = RLUpdate(theta(optCount-1,:),Vs,Vsn,x0,u(simCount-1,:)',nabla,MPCParam.R,Q,RLParam);
                
                Q = reshape(theta(optCount,1:nx*nx),2,2)';                    
                A = reshape(theta(optCount,nx*nx+1:2*nx*nx),2,2)';                  
                E = reshape(theta(optCount,2*nx*nx+1:2*nx*nx+nx),2,1);
                printState(theta(optCount,:),(simCount-1)*h,x0,timelabel,statelabel,Parameternumlabel,TD(optCount),u(simCount-1,:)')
            else
                nablan = numericalGradiant(MPCParam,"F",nums,Plqr,theta(optCount,:)');      % Check that KKT not violated
                KKT = numericalGradiant(MPCParam,"KKTF",nums,Plqr,theta(optCount,:)');      % Calculate derivative
            end
%----------------Error Checks-------------------------------
            if max(KKT)> opts.ipopt.acceptable_tol
                fprintf("Abo    rted due to an error in estimated KKT conditions\n");
                fprintf( ['The Prime conditions should be zero but is : ' repmat('% -6.4f  |', 1, size(KKT,2)) '\n'], KKT')                
                break;
            elseif det(Q)<0.001 || det(A)<0.001
                fprintf("Aborted due to loss of convexity or stability\n");          
                break;
            end            
 %---------------Apply the input, and move to next state-------------------

            xopt((optCount-1)*nx + 1:(optCount-1)*nx +nx ,1:N+1) = xx;
            uopt((optCount-1)*nu + 1:(optCount-1)*nu +nu ,1:N) = uu;            
            x = RK4(x,uu(:,1),h,simCount);
            u(simCount) = uu(:,1);            

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
    out.TD = TD(1:optCount); out.optCount = optCount;
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
   
function [timelabel,statelabel,Parameterlabel,Parameternumlabel]= printStateInit(theta,t,x,RLParam)
    delete(findall(0,'Name','progress'))
    fig = uifigure('Name', 'progress','Position',[680 866 619 112]);
    
    g = uigridlayout(fig,[4 1]);
    timelabel = uilabel(g,'text',sprintf("Simulation time: % -3.3f\n",t));
    timelabel.Layout.Row = 1;
    timelabel.Layout.Column = 1;

    statelabel = uilabel(g,'text',sprintf("Current State  : % -4.2f % -4.2f \n", x));
    statelabel.Layout.Row = 2;
    statelabel.Layout.Column = 1;



    Parameterlabel = uilabel(g,'text', sprintf( ['Parameters     : ' repmat(' %-5s |', 1, size(RLParam.theta,1)) '\n'], RLParam.theta'));
    Parameterlabel.Layout.Row = 3;
    Parameterlabel.Layout.Column = 1;
    % 
    Parameternumlabel = uilabel(g,'text',sprintf( ['Parameters Val : ' repmat('% -6.4f  |', 1, size(theta,2)) '\n'], theta'));
    Parameternumlabel.Layout.Row = 4;
    Parameternumlabel.Layout.Column = 1;
    pause(1)
end

function printState(theta,t,x,timelabel,statelabel,Parameternumlabel,TD,u)
    timelabel.Text = sprintf("Simulation time: % -3.3f \t Temporal error: % -3.3f ",t,TD);
    statelabel.Text = sprintf("Current State  : % -4.2f % -4.2f \t Current Input: % -4.2f", x,u);
    Parameternumlabel.Text = sprintf( ['Parameters Val : ' repmat('% -6.4f  |', 1, size(theta,2)) '\n'], theta');

end

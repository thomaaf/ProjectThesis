%% 
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

	[nlpProb,opts,args] = NLPsetup(Model,MPCParam,RLParam,InitParam);
	
	
%% Options
	%[opts,args] = casadiOptions(Model,MPCParam,RLParam,InitParam);
	solver = casadi.nlpsol('solver', 'ipopt', nlpProb,opts);   

%% Simulation loop
% Reshape counts row-wise.so for col; for row
%------Output Variables-----
	x       = zeros(tspan/h,nx);                    %Matrix of resulting states
	u       = zeros(tspan/h,nu);                     %Matrix of resulting inputs
	V       = zeros(tspan/h,1); 
	nabla   = zeros(tspan/h,size(RLParam.theta,1));      
	theta   = zeros(tspan/h,size(RLParam.theta,1)); %Matrix of theta variables 
	TD      = zeros(tspan/h,1);                     %Vector of the Temporal differences
	xopt    = zeros(tspan/h*nx,N+1);                %Matrix of all optimal trajetories at each optitime t
	uopt    = zeros(tspan/h*nu,N);                  %Matrix of all optimal inputs at each optitime t      
	numsopt = zeros(tspan/h*nx,size(MPCParam.vars,1));
	reward  = zeros(tspan/h,1);
	target  = zeros(tspan/h,1);


%----Misc initialization------
	args.X0 = zeros(nx,N+1);                              %Initial search area for X variables
	args.S0 = zeros(nx,N)
	args.U0 = zeros(nu,N);                               %Initial search area for U variables
	[timelabel,statelabel,~,Parameternumlabel]= printStateInit(theta(1,:),0,x0,RLParam); 
	%x(1,:) = x0;
	simcount =1;                            %Counter for tracking simulation iterations 
	inputTime = T/N;                        %Timer for when new optimalization is due.
	optCount = 0;                           %Counter for tracking optimizations
	[~,Plqr,~] = lqr(Model.A,Model.B,MPCParam.Q,MPCParam.R,[]);
	
%-----Calculations of initial values-----------    
% theta = [V0;xsub;xtop;E;B;f;A]:
	theta_s = [MPCParam.V0;Model.xsub;Model.xtop;Model.E;Model.B;MPCParam.F;Model.A(:)]                  
	
	s = x0;
	[Plqr, optNums,Vs,a] = solveOpt(solver,args,MPCParam,Model,theta_s,s,xRef);    
	
	[nabla_s,nums] = numericalGradiant(MPCParam,"thetaGradFunc",optNums,Plqr,theta_s,s);
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

	%Reshaping the storage matrix s.t it is von the form x1:;x2:;...
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
	A = reshape(theta(end+1 - nx^2:end),nx,nx);
	B = reshape(theta(end+1-nx^2 - 2*nx - nu:end-nx^2 - nx - nu),nx,nu);
	[~,Plqr,~] = dlqr(A,B,MPCParam.Q,MPCParam.R,[]);


	args.p = [Plqr(:);theta;s];   
	
	args.x0 = [args.X0(:);args.S0(:);args.U0(:)];       % Set the search space for next opti in args.x0
%---------------solve next optimization-----------------------------------            
	sol = solver('x0',args.x0, 'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
	Vs = full(sol.f);
	uu = reshape(full(sol.x(nx*(N+1)+1 + nx*N:end)),nu,N)
	ss = reshape(full(sol.x(nx*(N+1)+1:nx*(N+1) + nx*(N))),nx,N)	% Extract inputs
	xx = reshape(full(sol.x(1:nx*(N+1))),nx,N+1)    				% Extract states
	nums = full([sol.x;sol.lam_g;sol.lam_x]);  % Extract all numerical data
	a = uu(1);
end


function [nlpProb,opts,args] = NLPsetup(Model,MPCParam,RLParam,InitParam)
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
	S  = casadi.SX.sym('S',nx,N);   %Slack decision variable 
	U  = casadi.SX.sym('U',nu,N);   %Input/decision variables
	P = casadi.SX.sym('P',MPCParam.nSym + nx);    %Inital and reference values
	
%----------Defining the RL-parameters--------------   
	Plqr = reshape(P(1:nx*nx),nx,nx)										;%1:4
	V0   = reshape(P(nx*nx+1:nx*nx+1),1,1)									;%5
	xsub = reshape(P(nx*nx+2       		  :nx*nx+1 + nx),nx,1)  			;%6:7
	xtop = reshape(P(nx*nx+2 +   nx		  :nx*nx+1 + 2*nx),nx,1)			;%7:8
	b    = reshape(P(nx*nx+2 + 2*nx		  :nx*nx+1 + 3*nx),nx,1)			;%9:10
	B    = reshape(P(nx*nx+2 + 3*nx 	  :nx*nx+1 + 4*nx),nx,1)     		;%12:13
	f    = reshape(P(nx*nx+2 + 4*nx 	  :nx*nx+1 + 5*nx + nu),nx+nu,1) 	;%14:16
	A 	 = reshape(P(nx*nx+2 + 5*nx + nu  :nx*nx*2+1 + 5*nx + nu),nx,nx) 	;%17:20
	X0 	 = reshape(P(nx*nx*2+2 + 5*nx + nu:nx*nx*2+1 + 6*nx + nu),nx,1)		;%21:22
	%[Plqr(:);V0;xsub;xtop;b;B;f;A(:);X0]
	%Plqr = [0 0; 0 0];

%----------Discretize matricies-----------

%----------Setup of NLP--------------    
%Calculation of stagecost, Objective function, and constraints
	% Sum from k = 0:N-1 <-> k = 1:N
    g0 = X(:,1) - X0;
    g1 = X(:,2:N+1) - (A*X(:,1:N)+ B*U(:,1:N) + b);
    g2l = [0;1] - xsub + S(:,1:N) + X(:,1:N) ;
    g2t = [1;1] + xtop + S(:,1:N) - X(:,1:N) ;

    obj1 = f'*[X(:,1:N);U(:,1:N)];
    obj2 = 0.5*gamma.^(0:N-1).*(sum(X(:,1:N).^2) + 0.5*U(:,1:N).^2 + Model.w'*S(:,1:N));
    obj3 = V0 + gamma^N/2*X(:,N+1)'*Plqr*X(:,N+1);
    

	g = [g0;g1(:);g2l(:);g2t(:)];
	obj = sum(obj1) + sum(obj2) + sum(obj3); 


% Define Optimization variables and the nlpProblem
	OPTVariables = [X(:);S(:);U(:)];
	nlpProb = struct('f',obj,'x',OPTVariables, 'g',g,'p',P);
%--------Arguments & Options------------
%Options
    opts = struct;
    opts.ipopt.max_iter = 100;
    opts.ipopt.print_level = 0;
    opts.print_time = 0;
    opts.ipopt.acceptable_tol = 1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;
    opts.calc_multipliers = 0; 
% lbg&ubg = lower and upper bounds on the equality constraint
    Xlb = MPCParam.Xlb; Xub = MPCParam.Xub;
    Ulb = MPCParam.Ulb; Uub = MPCParam.Uub; 

    %lbg =  casadi.SX(size(g,1),1);
	lbg = zeros(size(g(:),1),1);
	ubg = [zeros(size(g0(:),1) + size(g1(:),1),1);inf(size(g2l(:),1)*2,1)];
    args = struct('lbg',lbg,'ubg',ubg,'lbx',-inf(size(OPTVariables,1),1),'ubx',inf(size(OPTVariables,1),1) );
%lbx&ubx = lower and upper bounds on the decision states;
%Checking for any upper or lower bounds, and adding them to args if
%existing.
    for i = 1:nx
        args.lbx(i+nx*(N+1) + nu*N:nx:nx*(N) + nx*(N+1) + nu*N,1) = -inf;
        args.ubx(i+nx*(N+1) + nu*N:nx:nx*(N) + nx*(N+1) + nu*N,1) = inf; 
        if ~isempty(Xlb) % Lower bounds on X
            args.lbx(i:nx:nx*(N+1),1) = Xlb(i); 
        end
        if ~isempty(Xub) % Upper bounds on X
            args.ubx(i:nx:nx*(N+1),1) = Xub(i);          
        end        
    end
    for i  = 1:nu
        if ~isempty(Ulb) % Lower bounds on U
            args.lbx(nx*(N+1) + nx*N+i:nu:nx*(N+1) + nx*N + nu*N,1) = Ulb(i); 
        end
        if ~isempty(Uub) % Upper bounds on U
            args.ubx(nx*(N+1) + nx*N+i:nu:nx*(N+1) + nx*N + nu*N,1) = Uub(i);           
        end       
    end
  
end
   
	


function [RLdata,numdata] = Simulation(Model,MPC,RL,Init,nlpProb,args,opts,nlpProbSym)
	nx = Model.nx; nu = Model.nu; N = MPC.N;
	numdata = zeros(Init.tspan,size(nlpProbSym.vars,1));
	RLdata = struct( 'theta',zeros(Init.tspan-1,size(RL.theta,1))...
					,'nabla',zeros(Init.tspan-1,size(RL.theta,1))...
					,'V',zeros(Init.tspan-1,1),'TD',zeros(Init.tspan-1,1)...
					,'R',zeros(Init.tspan-1,1),'Target',zeros(Init.tspan-1,1));

	solver = casadi.nlpsol('solver', 'ipopt', nlpProb,opts);   
	theta = RL.Theta;

%Solve initial problem
	args.X0 = zeros(nx,N+1); args.S0 = zeros(nx,N); args.U0 = zeros(nu,N);	
	args.x0 = [args.X0(:);args.S0(:);args.U0(:)]; 
	[nums,V,a] = solveOpt(Model,MPC,solver,args,theta,Init.X0);

%Initial conditions of problem-
	s = Init.X0;
	nabla = nlpProbSym.dLdtheta(nums)';
	t = 1



	while t < Init.tspan
		sn = progress(s,0);

 		[numsn,Vn,an] = solveOpt(Model,MPC,solver,args,theta,sn);
 		if abs(max(nlpProbSym.dLdx(numsn))) >1e-8
 			fprintf("Breach of KKT")
 			disp (nlpProbSym.dLdx(numsn))
 			break;
 		end 
 		nablan = nlpProbSym.dLdtheta(numsn)';
 		[R,target,TD,~] = RLupdate(RL,s,a,V,Vn,nabla,theta);


 		numdata(t,:) = nums;
 		RLdata.theta(t,:) = theta; RLdata.nabla(t,:) = nabla;
 		RLdata.V(t,:) = V; RLdata.TD(t,:) = TD;
		RLdata.R(t,:) = R; RLdata.Target(t,:) = target;

		s = sn; a = an;
		nums = numsn; 
		nabla = nablan;
		V = Vn

		t = t + 1


	end
end

function sn = progress(s,a)
	sn = [0.9 0.35;0 1.1]*s + [0.0813;0.2]*a +  [-1e-1*((rand));0 ];
end

function [nums,V,a] = solveOpt(Model,MPC,solver,args,theta,s)
	nx = Model.nx; nu = Model.nu; N = MPC.N;

	A = reshape(theta(13:16),nx,nx);
	B = reshape(theta(8:9),nx,1);
	[~,Plqr,~] = dlqr(A,B,MPC.Q,MPC.R,[]);

	args.p = [Plqr(:);theta;s];
	args.x0 = [args.X0(:);args.S0(:);args.U0(:)]; 
	sol = solver('x0',args.x0, 'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
	optNums = full([sol.x;sol.lam_g;sol.lam_x(end-nu*N+1:end)]);
	nums = [optNums;Plqr(:);theta;s];	
	a = full(sol.x(end-nu*N+1));
	V = full(sol.f);
end

function [R,target,TD,theta] = RLupdate(RL,s,a,V,Vn,nabla,theta)
	R = 0.5*s'*s + 0.25*a'*a;
	target = R + RL.gamma*Vn;
	TD = target - V;
	nabla
	theta = theta + RL.alpha*TD*nabla;
end
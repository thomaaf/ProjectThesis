function [RLdata,numdata] = Simulation(Model,MPC,RL,Init,nlpProb,args,opts,nlpProbSym)
	nx = Model.nx; nu = Model.nu; N = MPC.N;
	numdata = zeros(Init.tspan-1,size(nlpProbSym.vars,1));
	RLdata = struct( 'theta',zeros(Init.tspan-1,size(RL.theta,1))...
					,'nabla',zeros(Init.tspan-1,size(RL.theta,1))...
					,'V',zeros(Init.tspan-1,1),'TD',zeros(Init.tspan-1,1)...
					,'R',zeros(Init.tspan-1,1),'Target',zeros(Init.tspan-1,1));

	solver = casadi.nlpsol('solver', 'ipopt', nlpProb,opts);   
	theta = RL.Theta;
    
%Solve initial problem
	args.X0 = zeros(nx,N+1); args.S0 = zeros(nx,N+1); args.U0 = zeros(nu,N);	
	args.x0 = [args.X0(:);args.S0(:);args.U0(:)]; 
	[nums,V,a,args] = solveOpt(Model,MPC,solver,args,theta,Init.X0,nlpProbSym);
    nlpProbSym.dLdx(nums)
%Initial conditions of problem-
	s = Init.X0;
	nabla = nlpProbSym.dLdtheta(nums)';
	t = 1



	while t < Init.tspan
		sn = progress(s,a);

 		[numsn,Vn,an,args] = solveOpt(Model,MPC,solver,args,theta,sn,nlpProbSym);
 		if abs(max(nlpProbSym.dLdx(numsn))) >1e-8
 			fprintf("Breach of KKT")
 			disp (nlpProbSym.dLdx(numsn))
 			break;
 		end 
 		nablan = nlpProbSym.dLdtheta(numsn)';
 		[R,target,TD,theta] = RLupdate(RL,s,a,V,Vn,nabla,theta);

 		numdata(t,:) = nums;
 		
 		RLdata.theta(t,:) = theta; RLdata.nabla(t,:) = nabla;
 		RLdata.V(t,:) = V; RLdata.TD(t,:) = TD;
		RLdata.R(t,:) = R; RLdata.Target(t,:) = target;

		s = sn; a = an;
		nums = numsn; 
		nabla = nablan;
		V = Vn;

		t = t + 1


	end
	RLdata = [array2table(RLdata.theta(1:t-1,:),'Variablenames',string(RL.theta))...
			 ,array2table(RLdata.nabla(1:t-1,:),'Variablenames',repmat("lam_",16,1) + string(RL.theta))...
			 ,array2table(RLdata.V(1:t-1,:),'Variablenames',"V"),array2table(RLdata.TD(1:t-1,:),'Variablenames',"TD")...
			 ,array2table(RLdata.R(1:t-1,:),'Variablenames',"R"),array2table(RLdata.Target(1:t-1,:),'Variablenames',"Target")];	
	numdata = array2table(numdata(1:t-1,:), 'VariableNames',string(nlpProbSym.vars));
end

function sn = progress(s,a)
	sn = [0.9 0.35;0 1.1]*s + [0.0813;0.2]*a +  [-1e-1*rand;0 ];
end

function [nums,V,a,args] = solveOpt(Model,MPC,solver,args,theta,s,nlpProbSym)
	nx = Model.nx; nu = Model.nu; N = MPC.N;

	A = reshape(theta(13:16),nx,nx);
	B = reshape(theta(8:9),nx,1);
	[~,Plqr,~] = dlqr(A,B,MPC.Q,MPC.R,[]);

	args.p = [Plqr(:);theta;s];
	args.x0 = [args.X0(:);args.S0(:);args.U0(:)]; 
	sol = solver('x0',args.x0, 'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
	optNums = full([sol.x;sol.lam_g;sol.lam_x(23:44)]);
	args.x0 = full(sol.x);
	nums = [optNums;Plqr(:);theta;s];	
	a = full(sol.x(end-nu*N+1));
	V = nlpProbSym.f(nums);


end

function [R,target,TD,theta] = RLupdate(RL,s,a,V,Vn,nabla,theta)
	R = 0.5*(s'*s) + 0.25*(a'*a);
	target = R + RL.gamma*Vn;
	TD =  target - V;
	theta = theta + RL.alpha*TD*nabla.*RL.learn;
end